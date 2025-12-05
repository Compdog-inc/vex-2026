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

enum class WaypointMode
{
    FullStop,
    LinearVelocity,
    LinearAcceleration
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
    RuckigAlign(const std::function<KinematicState()> &currentStateSupplier,
                const std::function<RuckigAlignState()> &targetStateSupplier,
                const std::function<void(const ChassisSpeeds &speeds)> &robotRelativeDriveConsumer,
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
    WaypointMode waypoint_mode = WaypointMode::LinearAcceleration;

    std::function<KinematicState()> currentStateSupplier;
    std::function<RuckigAlignState()> targetStateSupplier;
    std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer;

    bool resetTrajectory;

    static constexpr double VELOCITY_TOLERANCE_MULTIPLIER = 5.0;
    static constexpr double VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD = 0.05; // meters
    // slightly larger threshold to avoid deceleration oscillation
    static constexpr double VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD_BYPASS = 0.08; // meters
    static constexpr double VELOCITY_MODE_BYPASS_VELOCITY_ERROR_THRESHOLD = 1.0;      // m/s

    static constexpr double DECELERATION_ACCEL_THRESHOLD = 0.9; // meters/s^2

    static constexpr double CURRENT_STATE_MULTIPLIER =
        0.5; // scales down the Ruckig current velocity/accel initialization for faster deceleration

    static constexpr double MAX_VELOCITY = 1.064;           // m/s
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

#endif // RUCKIGALIGN_H