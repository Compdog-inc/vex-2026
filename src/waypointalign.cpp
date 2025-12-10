#include "waypointalign.h"

std::vector<Pose2d> WaypointAlign::createWaypointsToTarget(const Pose2d &target, const std::vector<Transform2d> &targetTransforms)
{
    std::vector<Pose2d> waypoints;

    for (Transform2d transform : targetTransforms)
    {
        waypoints.push_back(target + transform);
    }
    waypoints.push_back(target);

    return waypoints;
}

KinematicState WaypointAlign::getKinematicStateForWaypoint(
    const Pose2d &previousWaypoint,
    const Pose2d &currentWaypoint,
    const Pose2d &nextWaypoint,
    const std::array<double, 3> &maxVelocity,
    const std::array<double, 3> &maxAcceleration,
    bool useAcceleration)
{

    double px = previousWaypoint.translation.x;
    double py = previousWaypoint.translation.y;
    double pr = previousWaypoint.rotation.value;

    double cx = currentWaypoint.translation.x;
    double cy = currentWaypoint.translation.y;
    double cr = MathUtil::closestTarget(
        pr, MathUtil::normalizeAngle(currentWaypoint.rotation.value));

    double nx = nextWaypoint.translation.x;
    double ny = nextWaypoint.translation.y;
    double nr = MathUtil::closestTarget(
        cr, MathUtil::normalizeAngle(nextWaypoint.rotation.value));

    double dpx = cx - px;
    double dpy = cy - py;
    double dpr = cr - pr;
    double dnx = nx - cx;
    double dny = ny - cy;
    double dnr = nr - cr;

    double lp = hypot(dpx, dpy);
    double ln = hypot(dnx, dny);
    double t = lp < ln ? lp / (2 * ln) : (1 - ln / (2 * lp));

    std::array<double, 3> velocity = calculateVelocity(dpx, dpy, dpr, dnx, dny, dnr, t, maxVelocity);
    std::array<double, 3> acceleration =
        useAcceleration ? calculateAcceleration(dnx, dny, dnr, maxAcceleration) : std::array<double, 3>{0, 0, 0};

    return KinematicState{
        {cx, cy, cr}, // position
        velocity,     // velocity
        acceleration  // acceleration
    };
}

std::array<double, 3> WaypointAlign::calculateVelocity(
    double dpx,
    double dpy,
    double dpr,
    double dnx,
    double dny,
    double dnr,
    double t,
    const std::array<double, 3> &maxVelocity)
{
    double dx = dnx * t + dpx * (1 - t);
    double dy = dny * t + dpy * (1 - t);
    double dr = dnr * t + dpr * (1 - t);
    dr = MathUtil::clamp(dr, -maxVelocity[2], maxVelocity[2]);

    double dist = hypot(dx, dy);
    if (dist > 1e-6)
    {
        double maxV = hypot(maxVelocity[0], maxVelocity[1]);
        if (fabs(dr) > 1e-6)
        {
            maxV /= fabs(dr);
        }
        if (dist > maxV)
        {
            dx = (dx / dist) * maxV;
            dy = (dy / dist) * maxV;
        }
    }
    else
    {
        dx = 0.0;
        dy = 0.0;
    }

    dx *= 1.05;
    dy *= 1.05;

    return {dx, dy, dr};
}

std::array<double, 3> WaypointAlign::calculateAcceleration(double dnx, double dny, double dnr, const std::array<double, 3> &maxAcceleration)
{
    dnr = MathUtil::clamp(dnr, -maxAcceleration[2], maxAcceleration[2]);
    double distA = hypot(dnx, dny);

    if (distA > 1e-6)
    {
        double maxA = hypot(maxAcceleration[0], maxAcceleration[1]);
        if (fabs(dnr) > 1e-6)
        {
            maxA /= fabs(dnr);
        }
        if (distA > maxA)
        {
            dnx = (dnx / distA) * maxA;
            dny = (dny / distA) * maxA;
        }
    }
    else
    {
        dnx = 0.0;
        dny = 0.0;
    }

    return {dnx, dny, dnr};
}

int WaypointAlign::getCurrentWaypoint(const std::vector<Pose2d> &waypoints, const KinematicState &currentState)
{
    double px = currentState.position[0];
    double py = currentState.position[1];

    double minDist = std::numeric_limits<double>::infinity();
    int closestWaypoint = -1;

    for (int i = 0; i < waypoints.size() - 1; i++)
    {
        int waypointIndex = i;

        Pose2d wp = waypoints[i];
        Pose2d wpn = waypoints[i + 1];
        double wx = wp.translation.x;
        double wy = wp.translation.y;
        double wnx = wpn.translation.x;
        double wny = wpn.translation.y;

        double t = ((px - wx) * (wnx - wx) + (py - wy) * (wny - wy)) / ((wnx - wx) * (wnx - wx) + (wny - wy) * (wny - wy));
        if (i == 0 && t < 0)
        {
            waypointIndex = -1;
        }
        t = MathUtil::clamp(t, 0, 1);
        double cx = wx + t * (wnx - wx);
        double cy = wy + t * (wny - wy);
        double dist = hypot(cx - px, cy - py);
        if (dist < minDist)
        {
            minDist = dist;
            closestWaypoint = waypointIndex;
        }
    }

    return closestWaypoint;
}

/**
 * Aligns the robot to a target pose. The robot will move to the target pose and stop there.
 * @param target The target pose to align to
 * @param timeout The timeout for the alignment
 * @param controlModifier The control modifier to use for the alignment
 * @return A command that aligns the robot to the target
 */

Command *WaypointAlign::align(
    const Pose2d &target,
    double timeout,
    std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
    std::function<KinematicState()> currentStateSupplier,
    Subsystem *requirementSubsystem)
{
    return align({target}, 0, 0, true, {timeout}, robotRelativeDriveConsumer, currentStateSupplier, requirementSubsystem);
}

/**
 * Aligns the robot with a series of waypoints. The robot will move through each waypoint in order,
 * starting from {@code startWaypoint} and ending at {@code endWaypoint}. If {@code stopAtEndWaypoint}
 * is true, the robot will stop at the end waypoint, otherwise it will continue moving.
 * @param waypoints The waypoints to align with
 * @param startWaypoint The index of the waypoint to start at (inclusive)
 * @param endWaypoint The index of the waypoint to end at (inclusive)
 * @param stopAtEndWaypoint Whether to stop at the end waypoint (NOTE!: if the end waypoint is the last waypoint, the robot will always stop there)
 * @param waypointTimeouts The timeouts for each waypoint
 * @param controlModifier The control modifier to use for the alignment
 * @return A command that aligns the robot with the waypoints
 */

Command *WaypointAlign::align(
    const std::vector<Pose2d> &waypoints,
    int startWaypoint,
    int endWaypoint,
    bool stopAtEndWaypoint,
    const std::vector<double> &waypointTimeouts,
    std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
    std::function<KinematicState()> currentStateSupplier,
    Subsystem *requirementSubsystem)
{
    RuckigAlignGroup<WaypointData> waypointGroup = RuckigAlignGroup<WaypointData>(
        currentStateSupplier,
        robotRelativeDriveConsumer,
        requirementSubsystem);

    return align(
               waypoints,
               startWaypoint,
               endWaypoint,
               stopAtEndWaypoint,
               waypointTimeouts,
               waypointGroup,
               currentStateSupplier)
        .build();
}

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

RuckigAlignGroup<WaypointAlign::WaypointData> WaypointAlign::align(
    const std::vector<Pose2d> &waypoints,
    int startWaypoint,
    int endWaypoint,
    bool stopAtEndWaypoint,
    const std::vector<double> &waypointTimeouts,
    RuckigAlignGroup<WaypointData> &waypointGroup,
    std::function<KinematicState()> currentStateSupplier)
{
    if (!validateAlignParameters(waypoints, startWaypoint, endWaypoint, waypointTimeouts))
        return waypointGroup;

    for (int i = startWaypoint; i <= endWaypoint; i++)
    {
        addWaypointToGroup(
            waypoints,
            i,
            endWaypoint,
            stopAtEndWaypoint,
            waypointTimeouts,
            waypointGroup,
            currentStateSupplier);
    }

    return waypointGroup;
}

bool WaypointAlign::validateAlignParameters(
    const std::vector<Pose2d> &waypoints, int startWaypoint, int endWaypoint, const std::vector<double> &waypointTimeouts)
{
    if (waypoints.size() > waypointTimeouts.size())
    {
        PANIC("Waypoints and waypoint timeouts must have the same length");
        return false;
    }

    if (startWaypoint < 0 || startWaypoint >= waypoints.size())
    {
        PANIC("startWaypoint is out of bounds");
        return false;
    }

    if (endWaypoint < 0 || endWaypoint >= waypoints.size())
    {
        PANIC("endWaypoint is out of bounds");
        return false;
    }

    return true;
}

void WaypointAlign::addWaypointToGroup(
    const std::vector<Pose2d> &waypoints,
    int index,
    int endWaypoint,
    bool stopAtEndWaypoint,
    const std::vector<double> &waypointTimeouts,
    RuckigAlignGroup<WaypointData> &waypointGroup,
    std::function<KinematicState()> currentStateSupplier)
{
    waypointGroup.addAlign(
        [waypoints, index, currentStateSupplier]()
        { return createWaypointData(waypoints, index, currentStateSupplier()); },
        [index, endWaypoint, stopAtEndWaypoint, waypoints](const WaypointData &data)
        { return createRuckigAlignState(data, index, endWaypoint, stopAtEndWaypoint, waypoints.size()); },
        waypointTimeouts[index]);
}

WaypointAlign::WaypointData WaypointAlign::createWaypointData(
    const std::vector<Pose2d> &waypoints,
    int index,
    const KinematicState &currentState)
{
    const KinematicState fullStopState = createFullStopState(waypoints[index]);
    const KinematicState velocityState = createVelocityState(waypoints, index, currentState);
    return {fullStopState, velocityState};
}

KinematicState WaypointAlign::createFullStopState(const Pose2d &waypoint)
{
    return KinematicState{
        {waypoint.translation.x, waypoint.translation.y, waypoint.rotation.value},
        {0, 0, 0},
        {0, 0, 0}};
}

KinematicState WaypointAlign::createVelocityState(
    const std::vector<Pose2d> &waypoints,
    int index,
    const KinematicState &currentState)
{
    if (index == waypoints.size() - 1)
    {
        return createFullStopState(waypoints[index]);
    }

    Pose2d previousWaypoint = index == 0
                                  ? Pose2d{
                                        Translation2d{currentState.position[0],
                                                      currentState.position[1]},
                                        Rotation2d{currentState.position[2]}}
                                  : waypoints[index - 1];

    return getKinematicStateForWaypoint(
        previousWaypoint,
        waypoints[index],
        waypoints[index + 1],
        {RuckigAlign::MAX_VELOCITY,
         RuckigAlign::MAX_VELOCITY,
         RuckigAlign::MAX_ANGULAR_VELOCITY},
        {RuckigAlign::MAX_ACCELERATION,
         RuckigAlign::MAX_ACCELERATION,
         RuckigAlign::MAX_ANGULAR_ACCELERATION},
        true);
}

RuckigAlignState WaypointAlign::createRuckigAlignState(
    const WaypointData &data, int index, int endWaypoint, bool stopAtEndWaypoint, int waypointsSize)
{
    bool shouldStop = index == waypointsSize - 1 || (stopAtEndWaypoint && index == endWaypoint);
    if (shouldStop)
    {
        return RuckigAlignState{data.fullStopState, AlignMode::Position};
    }
    else
    {
        return RuckigAlignState{data.velocityState, AlignMode::Velocity};
    }
}

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
 * @param constraints The maximum kinematic constraints to use
 * @param controlModifier The control modifier to use for the alignment
 * @return A command that aligns the robot with the waypoints and runs the command at the specified waypoint
 */
Command *WaypointAlign::alignWithCommand(
    const std::vector<Pose2d> &waypoints,
    const std::vector<double> &waypointTimeouts,
    int commandStartWaypoint,
    int commandEndWaypoint,
    Command *performCommand,
    std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
    std::function<KinematicState()> currentStateSupplier,
    Subsystem *requirementSubsystem)
{
    if (waypoints.size() > waypointTimeouts.size())
    {
        PANIC("Waypoints and waypoint timeouts must have the same length");
        return nullptr;
    }

    return Commands::defer(
        [waypoints,
         waypointTimeouts,
         currentStateSupplier,
         robotRelativeDriveConsumer,
         requirementSubsystem,
         performCommand,
         commandEndWaypoint,
         commandStartWaypoint]()
        {
            RuckigAlignGroup<WaypointData> waypointGroup = RuckigAlignGroup<WaypointData>(
                currentStateSupplier,
                robotRelativeDriveConsumer,
                requirementSubsystem);

            align(
                waypoints,
                std::min(commandEndWaypoint, getCurrentWaypoint(waypoints, currentStateSupplier()) + 1),
                commandEndWaypoint,
                false,
                waypointTimeouts,
                waypointGroup.newGroup(),
                currentStateSupplier);
            align(
                waypoints,
                commandEndWaypoint + 1,
                waypoints.size() - 1,
                true,
                waypointTimeouts,
                waypointGroup.newGroup(),
                currentStateSupplier);

            bool *firstAlignFinished = new bool[1]{false};

            return waypointGroup
                .build(0)
                ->andThen([firstAlignFinished]()
                          { firstAlignFinished[0] = true; })
                /* if waypoint align finished we can assume we are at the end */
                ->alongWith(
                    {Commands::waitUntilCondition([waypoints, currentStateSupplier, commandStartWaypoint, firstAlignFinished]()
                                                  { return getCurrentWaypoint(waypoints, currentStateSupplier()) == commandStartWaypoint || firstAlignFinished[0]; })
                         ->andThen(performCommand)})
                ->andThen(waypointGroup.build(1));
        },
        {requirementSubsystem});
}