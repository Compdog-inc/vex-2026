#ifndef RUCKIGALIGN_H
#define RUCKIGALIGN_H

#include "rmath.h"
#include <ruckig/ruckig.hpp>
#include "commandscheduler.h"

using namespace ruckig;

enum class RobotControlMode
{
    PID,
    Feedforward,
    Combined
};

enum class AlignMode
{
    Position, // Full stop at target
    Velocity  // Keep moving at target velocity
};

enum class DeceleratingState
{
    NOT_DECELERATING,
    DECELERATING,
    DONE_DECELERATING
};

typedef struct KinematicState
{
    std::array<double, 3> position;
    std::array<double, 3> velocity;
    std::array<double, 3> acceleration;
} KinematicState;

typedef struct RuckigAlignState
{
    KinematicState kinematicState;
    AlignMode alignMode;
} RuckigAlignState;

class RuckigAlign : public Command
{
public:
    RuckigAlign(std::function<KinematicState()> currentStateSupplier,
                std::function<RuckigAlignState()> targetStateSupplier,
                std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
                bool resetTrajectory);

    static std::array<double, 3> getCurrentNewPosition();

    void initialize() override;
    void execute() override;

    bool isFinished() override;

    static void setLastAlignSuccessful(bool success);
    static bool wasLastAlignSuccessful();

    static KinematicState toKinematicState(const Pose2d &pose, const ChassisSpeeds &speeds, const ChassisSpeeds &accel);

    static void setup();

private:
    static double feedforward(double velocity, double acceleration, double jerk);

    static std::array<double, 3> processPoseForRuckig(const Pose2d &pose);
    static std::array<double, 3> chassisSpeedsToArray(const ChassisSpeeds &speeds);
    static std::array<double, 3> processChassisSpeeds(const ChassisSpeeds &speeds, const Pose2d &pose);

    static void setTargetState(const KinematicState &state);
    void applyAlignState(const RuckigAlignState &state);

    std::array<double, 3> getProcessedMaxAcceleration() const;
    std::array<double, 3> getProcessedMaxJerk() const;

    static void maximizeConstraints(const std::array<double, 3> &maxVelocity, const std::array<double, 3> &maxAcceleration);

    template <std::size_t _Nm>
    inline static std::array<double, _Nm> scaleDoubleArray(const std::array<double, _Nm> &array, double scale)
    {
        std::array<double, _Nm> scaled = {};
        for (int i = 0; i < array.size(); i++)
        {
            scaled[i] = array[i] * scale;
        }
        return scaled;
    }

    void reset();

    /**
     * Tight tolerance for position mode (final full-stop target)
     */
    static void setPositionModeTolerance();

    /**
     * Increases the tolerance for velocity mode
     * to insure smooth waypoint following (final target is still in position mode)
     */
    static void setVelocityModeTolerance();

    Result result = Result::Finished;

    std::array<double, 3> maxVelocity;
    std::array<double, 3> maxAcceleration;
    std::array<double, 3> maxDeceleration;
    std::array<double, 3> maxJerk;
    std::array<double, 3> maxDejerk;

    std::array<DeceleratingState, 3> isDecelerating = {
        DeceleratingState::NOT_DECELERATING,
        DeceleratingState::NOT_DECELERATING,
        DeceleratingState::NOT_DECELERATING};

    RobotControlMode control_mode = RobotControlMode::Combined;

    std::function<KinematicState()> currentStateSupplier;
    std::function<RuckigAlignState()> targetStateSupplier;
    std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer;

    bool resetTrajectory;

public:
    static constexpr double VELOCITY_TOLERANCE_MULTIPLIER = 5.0;
    static constexpr double VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD = 0.1; // meters
    // slightly larger threshold to avoid deceleration oscillation
    static constexpr double VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD_BYPASS = 0.15; // meters
    static constexpr double VELOCITY_MODE_BYPASS_VELOCITY_ERROR_THRESHOLD = 0.45;     // m/s

    static constexpr double DECELERATION_ACCEL_THRESHOLD = 0.9; // meters/s^2

    static constexpr double CURRENT_STATE_MULTIPLIER =
        0.5; // scales down the Ruckig current velocity/accel initialization for faster deceleration

    static constexpr double MAX_VELOCITY = 1.41;            // m/s
    static constexpr double MAX_ANGULAR_VELOCITY = 4.0;     // rad/s
    static constexpr double MAX_ACCELERATION = 1.5;         // m/s^2
    static constexpr double MAX_ANGULAR_ACCELERATION = 7.0; // rad/s^2
    static constexpr double MAX_DECELERATION = 2.0;         // m/s^2
    static constexpr double MAX_ANGULAR_DECELERATION = 8.0; // rad/s^2
    static constexpr double MAX_JERK = 3.0;                 // m/s^3
    static constexpr double MAX_ANGULAR_JERK = 7.0;         // rad/s^3
    static constexpr double MAX_DEJERK = 3.0;               // m/s^3
    static constexpr double MAX_ANGULAR_DEJERK = 7.0;       // rad/s^3

    static constexpr double TRANSLATIONAL_TOLERANCE = 0.01;                         // Meters
    static constexpr double TRANSLATIONAL_VELOCITY_TOLERANCE = 0.05;                // Meters/s
    static constexpr double ROTATIONAL_TOLERANCE = MathUtil::toRadians(1);          // Radians
    static constexpr double ROTATIONAL_VELOCITY_TOLERANCE = MathUtil::toRadians(5); // Radians/s
};

template <typename T>
class RuckigAlignGroup
{
public:
    /**
     * Create a RuckigAlignGroup
     */
    RuckigAlignGroup(std::function<KinematicState()> currentStateSupplier,
                     std::function<void(const ChassisSpeeds &speeds)> defaultRobotRelativeDriveConsumer,
                     Subsystem *requirementSubsystem)
        : currentStateSupplier(std::move(currentStateSupplier)),
          defaultRobotRelativeDriveConsumer(std::move(defaultRobotRelativeDriveConsumer)),
          requirementSubsystem(requirementSubsystem)
    {
    }

    /**
     * Start a new group of align states (used for splitting the group command into multiple)
     * @return this (for chaining)
     */
    RuckigAlignGroup<T> &newGroup()
    {
        return newGroup(defaultRobotRelativeDriveConsumer);
    }

    /**
     * Start a new group of align states (used for splitting the group command into multiple)
     * @param controlModifier the control modifier to use for this group
     * @return this (for chaining)
     */
    RuckigAlignGroup<T> &newGroup(std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer)
    {
        Group group;
        group.startIndex = states.size();
        group.robotRelativeDriveConsumer = robotRelativeDriveConsumer;
        groups.push_back(group);
        return *this;
    }

    /**
     * Add an align state to the group
     * @param initializer Function to initialize any parameters needed for the state
     * @param state Function to get the RuckigAlignState from the parameters
     * @param timeout Timeout for this align state
     * @return this (for chaining)
     */
    RuckigAlignGroup<T> &addAlign(std::function<T()> initializer,
                                  std::function<RuckigAlignState(const T &)> state,
                                  double timeout)
    {
        AlignEntry entry;
        entry.initializer = initializer;
        entry.state = state;
        entry.timeout = timeout;
        states.push_back(entry);
        return *this;
    }

    std::function<KinematicState()> getCurrentStateSupplier() const
    {
        return currentStateSupplier;
    }

    /**
     * Build the RuckigAlign command sequence for all groups
     * @return the command sequence
     */
    Command *build()
    {
        return build(0, groups.size() - 1);
    }

    /**
     * Build the RuckigAlign command sequence for a specific group
     * @param group the group index
     * @return the command sequence
     */
    Command *build(int group)
    {
        return build(group, group);
    }

    /**
     * Build the RuckigAlign command sequence for a range of groups
     * @param startGroup the starting group index (inclusive)
     * @param endGroup the ending group index (inclusive)
     * @return the command sequence
     */
    Command *build(int startGroup, int endGroup)
    {
        if (groups.empty())
        {
            return build({});
        }

        std::vector<int> indices;
        for (int g = startGroup; g <= endGroup; g++)
        {
            indices.push_back(g);
        }

        return build(indices);
    }

    /**
     * Build the RuckigAlign command sequence for a range of groups
     * @param groupIds the group indices to include
     * @return the command sequence
     */
    Command *build(const std::vector<int> &groupIds)
    {
        if (states.empty())
            return Commands::none();

        std::vector<Command *> commands;

        if (groups.empty())
        {
            // If no groups were defined, treat all states as a single group
            addStateCommands(defaultRobotRelativeDriveConsumer, 0, states.size(), commands);
        }
        else
        {
            for (int group : groupIds)
            {
                if (group < 0 || group >= groups.size())
                {
                    return nullptr;
                }

                int startIndex = groups[group].startIndex;
                int endIndex = (group + 1 < groups.size()) ? groups[group + 1].startIndex : states.size();
                addStateCommands(groups[group].robotRelativeDriveConsumer, startIndex, endIndex, commands);
            }
        }

        return Commands::sequence(commands);
    }

private:
    typedef struct AlignEntry
    {
        std::function<T()> initializer;
        std::function<RuckigAlignState(const T &)> state;
        double timeout;
    } AlignEntry;

    typedef struct Group
    {
        int startIndex;
        std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer;
    } Group;

    /**
     * Add states from startIndex (inclusive) to endIndex (exclusive) to the commands list
     * @param controlModifier the control modifier to use for these states
     * @param startIndex the starting state index (inclusive)
     * @param endIndex the ending state index (exclusive)
     * @param commands the list to add the commands to
     */
    void addStateCommands(std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer, int startIndex, int endIndex, std::vector<Command *> &commands)
    {
        for (int i = startIndex; i < endIndex; i++)
        {
            const int index = i;
            const AlignEntry entryCopy = states[index];
            const std::function<KinematicState()> currentStateSupplierCopy = this->currentStateSupplier;

            commands.push_back(Commands::defer(
                                   [currentStateSupplierCopy, entryCopy, robotRelativeDriveConsumer, index]()
                                   {
                                       const T param = entryCopy.initializer();
                                       RuckigAlign *align = new RuckigAlign(
                                           currentStateSupplierCopy,
                                           [entryCopy, param]()
                                           {
                                               return entryCopy.state(param);
                                           },
                                           robotRelativeDriveConsumer,
                                           index == 0);
                                       align->setOnHeap(true);
                                       return align;
                                   },
                                   {requirementSubsystem})
                                   ->withTimeout(entryCopy.timeout));
        }
    }

    std::vector<AlignEntry> states;
    std::vector<Group> groups;

    std::function<KinematicState()> currentStateSupplier;
    std::function<void(const ChassisSpeeds &speeds)> defaultRobotRelativeDriveConsumer;

    Subsystem *requirementSubsystem;
};

#endif // RUCKIGALIGN_H