#ifndef WAYPOINTALIGN_H
#define WAYPOINTALIGN_H

#include "ruckigalign.h"
#include "commandscheduler.h"

class WaypointAlign
{
public:
    typedef struct WaypointData
    {
        KinematicState fullStopState;
        KinematicState velocityState;
    } WaypointData;

    static std::vector<Pose2d> createWaypointsToTarget(const Pose2d &target, const std::vector<Transform2d> &targetTransforms);

    /**
     * Aligns the robot to a target pose. The robot will move to the target pose and stop there.
     * @param target The target pose to align to
     * @param timeout The timeout for the alignment
     * @param robotRelativeDriveConsumer The control modifier to use for the alignment
     * @return A command that aligns the robot to the target
     */
    static Command *align(
        const Pose2d &target,
        double timeout,
        std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
        std::function<KinematicState()> currentStateSupplier,
        Subsystem *requirementSubsystem);

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @param robotRelativeDriveConsumer The control modifier to use for the alignment
     * @return A command that aligns the robot with the waypoints
     */
    static Command *align(
        const std::vector<Pose2d> &waypoints,
        int startWaypoint,
        int endWaypoint,
        bool stopAtEndWaypoint,
        const std::vector<double> &waypointTimeouts,
        std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
        std::function<KinematicState()> currentStateSupplier,
        Subsystem *requirementSubsystem = nullptr);

    /**
     * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
     * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
     * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
     * @param waypoints The waypoints to align with
     * @param startWaypoint The index of the waypoint to start at (inclusive)
     * @param endWaypoint The index of the waypoint to end at (inclusive)
     * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
     * @param waypointTimeouts The timeouts for each waypoint
     * @param waypointGroup The RuckigAlignGroup to use for the alignment (useful for chaining multiple alignments together)
     * @return The RuckigAlignGroup with the waypoints added
     */
    static RuckigAlignGroup<WaypointData> align(
        const std::vector<Pose2d> &waypoints,
        int startWaypoint,
        int endWaypoint,
        bool stopAtEndWaypoint,
        const std::vector<double> &waypointTimeouts,
        RuckigAlignGroup<WaypointData> &waypointGroup,
        std::function<KinematicState()> currentStateSupplier);

    /**
     * Aligns the robot with a command that runs at a specific waypoint. The command will run
     * immediately when the robot reaches the waypoint specified by {@code commandStartWaypoint} and
     * will wait until the robot reaches the waypoint specified by {@code commandEndWaypoint} before
     * continuing to the next waypoint.
     *
     * @param waypoints The waypoints to align with
     * @param waypointTimeouts The timeouts for each waypoint
     * @param commandStartWaypoint The waypoint when to run the command
     * @param commandEndWaypoint The waypoint before which to wait for command to finish (if start is
     *     -1 and end is 0 then the command will run immediately and wait before entering the segment
     *     between first and second waypoint)
     * @param performCommand The command to run when at the right waypoint
     * @param controlModifier The control modifier to use for the alignment
     * @return A command that aligns the robot with the waypoints and runs the command at the specified waypoint
     */
    static Command *alignWithCommand(
        const std::vector<Pose2d> &waypoints,
        const std::vector<double> &waypointTimeouts,
        int commandStartWaypoint,
        int commandEndWaypoint,
        Command *performCommand,
        std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
        std::function<KinematicState()> currentStateSupplier,
        Subsystem *requirementSubsystem = nullptr);

private:
    static KinematicState getKinematicStateForWaypoint(const Pose2d &previousWaypoint,
                                                       const Pose2d &currentWaypoint,
                                                       const Pose2d &nextWaypoint,
                                                       const std::array<double, 3> &maxVelocity,
                                                       const std::array<double, 3> &maxAcceleration,
                                                       bool useAcceleration);

    static std::array<double, 3> calculateVelocity(double dpx,
                                                   double dpy,
                                                   double dpr,
                                                   double dnx,
                                                   double dny,
                                                   double dnr,
                                                   double t,
                                                   const std::array<double, 3> &maxVelocity);

    static std::array<double, 3> calculateAcceleration(double dnx, double dny, double dnr, const std::array<double, 3> &maxAcceleration);

    static int getCurrentWaypoint(const std::vector<Pose2d> &waypoints, const KinematicState &currentState);

    static bool validateAlignParameters(
        const std::vector<Pose2d> &waypoints, int startWaypoint, int endWaypoint, const std::vector<double> &waypointTimeouts);

    static void addWaypointToGroup(
        const std::vector<Pose2d> &waypoints,
        int index,
        int endWaypoint,
        bool stopAtEndWaypoint,
        const std::vector<double> &waypointTimeouts,
        RuckigAlignGroup<WaypointData> &waypointGroup,
        std::function<KinematicState()> currentStateSupplier);

    static WaypointData createWaypointData(
        const std::vector<Pose2d> &waypoints,
        int index,
        const KinematicState &currentState);

    static KinematicState createFullStopState(const Pose2d &waypoint);
    static KinematicState createVelocityState(
        const std::vector<Pose2d> &waypoints,
        int index,
        const KinematicState &currentState);

    static RuckigAlignState createRuckigAlignState(
        const WaypointData &data, int index, int endWaypoint, bool stopAtEndWaypoint, int waypointsSize);
};

#endif