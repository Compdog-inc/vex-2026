#include "ruckigalign.h"

#define PERIOD 0.01

static Ruckig<3> ruckigInstance(PERIOD);
static InputParameter<3> input{};
static OutputParameter<3> output{};

static AlignMode currentMode = AlignMode::Position;

static PIDController xPid{8.0, 0.0, 0.1, PERIOD};
static PIDController yPid{8.0, 0.0, 0.1, PERIOD};
static PIDController rPid{4.0, 0.0, 0.2, PERIOD};

static bool lastAlignSuccessful = false;

RuckigAlign::RuckigAlign(std::function<KinematicState()> currentStateSupplier,
                         std::function<RuckigAlignState()> targetStateSupplier,
                         std::function<void(const ChassisSpeeds &speeds)> robotRelativeDriveConsumer,
                         bool resetTrajectory)
    : currentStateSupplier(std::move(currentStateSupplier)),
      targetStateSupplier(std::move(targetStateSupplier)),
      robotRelativeDriveConsumer(std::move(robotRelativeDriveConsumer)),
      resetTrajectory(resetTrajectory)
{
    maxVelocity = {MAX_VELOCITY, MAX_VELOCITY, MAX_ANGULAR_VELOCITY};
    maxAcceleration = {MAX_ACCELERATION, MAX_ACCELERATION, MAX_ANGULAR_ACCELERATION};
    maxDeceleration = {MAX_DECELERATION, MAX_DECELERATION, MAX_ANGULAR_DECELERATION};
    maxJerk = {MAX_JERK, MAX_JERK, MAX_ANGULAR_JERK};
    maxDejerk = {MAX_DEJERK, MAX_DEJERK, MAX_ANGULAR_DEJERK};
}

std::array<double, 3> RuckigAlign::getCurrentNewPosition()
{
    return output.new_position;
}

void RuckigAlign::setup()
{
    rPid.enableContinuousInput(-M_PI, M_PI);
    setPositionModeTolerance();

    input.duration_discretization = DurationDiscretization::Discrete;
    input.synchronization = Synchronization::Phase;
    input.per_dof_synchronization = std::optional<std::array<Synchronization, 3>>(
        std::array<Synchronization, 3>{Synchronization::Phase, Synchronization::Phase, Synchronization::Time});
}

void RuckigAlign::initialize()
{
    if (!resetTrajectory)
    {
        isDecelerating.fill(DeceleratingState::DONE_DECELERATING);
    }

    input.max_velocity = maxVelocity;
    input.max_acceleration = maxAcceleration;
    input.max_jerk = maxJerk;

    if (resetTrajectory)
    {
        reset();
    }

    applyAlignState(targetStateSupplier());
    setLastAlignSuccessful(false);
}

void RuckigAlign::reset()
{
    KinematicState currentState = currentStateSupplier();

    input.current_position = currentState.position;
    input.current_velocity = scaleDoubleArray(currentState.velocity, CURRENT_STATE_MULTIPLIER);
    input.current_acceleration = scaleDoubleArray(currentState.acceleration, CURRENT_STATE_MULTIPLIER);

    xPid.reset();
    yPid.reset();
    rPid.reset();

    result = Result::Working;
    isDecelerating.fill(DeceleratingState::NOT_DECELERATING);
}

void RuckigAlign::execute()
{
    applyAlignState(targetStateSupplier());

    result = ruckigInstance.update(input, output);

    KinematicState currentState = currentStateSupplier();

    for (int i = 0; i < isDecelerating.size(); i++)
    {
        if (fabs(output.new_acceleration[i]) > DECELERATION_ACCEL_THRESHOLD && !MathUtil::signEq(output.new_velocity[i], output.new_acceleration[i]))
        {
            if (isDecelerating[i] == DeceleratingState::NOT_DECELERATING)
            {
                isDecelerating[i] = DeceleratingState::DECELERATING;
            }
        }
        else if (isDecelerating[i] == DeceleratingState::DECELERATING)
        {
            isDecelerating[i] = DeceleratingState::DONE_DECELERATING;
        }
    }

    double x_velocity = 0.0;
    double y_velocity = 0.0;
    double angular_velocity = 0.0;

    if (control_mode == RobotControlMode::PID || control_mode == RobotControlMode::Combined)
    {
        x_velocity += xPid.calculate(currentState.position[0], output.new_position[0]);
        y_velocity += yPid.calculate(currentState.position[1], output.new_position[1]);
        angular_velocity += rPid.calculate(currentState.position[2], output.new_position[2]);
    }

    if (control_mode == RobotControlMode::Feedforward || control_mode == RobotControlMode::Combined)
    {
        x_velocity += feedforward(output.new_velocity[0], output.new_acceleration[0], 0.0);
        y_velocity += feedforward(output.new_velocity[1], output.new_acceleration[1], 0.0);
        angular_velocity += feedforward(output.new_velocity[2], output.new_acceleration[2], 0.0);
    }

    output.pass_to_input(input);

    double ex = fabs(currentState.position[0] - output.new_position[0]);
    double ey = fabs(currentState.position[1] - output.new_position[1]);
    double er = fabs(MathUtil::closestTarget(
                         output.new_position[2], MathUtil::normalizeAngle(currentState.position[2])) -
                     output.new_position[2]);

    // If the error is too large, reset the trajectory to current position for more accurate motion
    if (ex * ex + ey * ey > 0.25 || er > 1.0)
    {
        reset();
    }

    robotRelativeDriveConsumer(
        ChassisSpeeds::fromFieldRelativeSpeeds(
            ChassisSpeeds{x_velocity, y_velocity, angular_velocity},
            Rotation2d{currentState.position[2]}));
}

double RuckigAlign::feedforward(double velocity, double acceleration, double jerk)
{
    const double kV = 1.0;
    const double kA = 0.1;
    const double kJ = 0.01;
    return kV * velocity + kA * acceleration + kJ * jerk;
}

bool RuckigAlign::isFinished()
{
    bool finished = false;

    if (currentMode == AlignMode::Position)
    {
        finished = result != Result::Working && xPid.atSetpoint() && yPid.atSetpoint() && rPid.atSetpoint();
    }
    else if (currentMode == AlignMode::Velocity)
    {
        KinematicState currentState = currentStateSupplier();
        double distanceToTarget = hypot(
            currentState.position[0] - input.target_position[0],
            currentState.position[1] - input.target_position[1]);

        double velocityError = hypot(
            input.current_velocity[0] - input.target_velocity[0],
            input.current_velocity[1] - input.target_velocity[1]);

        finished = distanceToTarget < (velocityError >= VELOCITY_MODE_BYPASS_VELOCITY_ERROR_THRESHOLD
                                           ? VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD_BYPASS
                                           : VELOCITY_MODE_DISTANCE_TO_TARGET_THRESHOLD) ||
                   result != Result::Working;
    }

    if (finished)
    {
        setLastAlignSuccessful(true);
    }
    return finished;
}

void RuckigAlign::setLastAlignSuccessful(bool success)
{
    lastAlignSuccessful = success;
}

bool RuckigAlign::wasLastAlignSuccessful()
{
    return lastAlignSuccessful;
}

/**
 * Tight tolerance for position mode (final full-stop target)
 */
void RuckigAlign::setPositionModeTolerance()
{
    xPid.setTolerance(TRANSLATIONAL_TOLERANCE, TRANSLATIONAL_VELOCITY_TOLERANCE);
    yPid.setTolerance(TRANSLATIONAL_TOLERANCE, TRANSLATIONAL_VELOCITY_TOLERANCE);
    rPid.setTolerance(ROTATIONAL_TOLERANCE, ROTATIONAL_VELOCITY_TOLERANCE);
    currentMode = AlignMode::Position;
}

/**
 * Increases the tolerance for velocity mode
 * to insure smooth waypoint following (final target is still in position mode)
 */
void RuckigAlign::setVelocityModeTolerance()
{
    xPid.setTolerance(
        TRANSLATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, std::numeric_limits<double>::infinity());
    yPid.setTolerance(
        TRANSLATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, std::numeric_limits<double>::infinity());
    rPid.setTolerance(
        ROTATIONAL_TOLERANCE * VELOCITY_TOLERANCE_MULTIPLIER, std::numeric_limits<double>::infinity());
    currentMode = AlignMode::Velocity;
}

std::array<double, 3> RuckigAlign::processPoseForRuckig(const Pose2d &pose)
{
    return {
        pose.translation.x,
        pose.translation.y,
        MathUtil::closestTarget(
            input.current_position[2],
            MathUtil::normalizeAngle(pose.rotation.value))};
}

std::array<double, 3> RuckigAlign::chassisSpeedsToArray(const ChassisSpeeds &speeds)
{
    return {speeds.vx, speeds.vy, speeds.omega};
}

std::array<double, 3> RuckigAlign::processChassisSpeeds(const ChassisSpeeds &speeds, const Pose2d &pose)
{
    return chassisSpeedsToArray(ChassisSpeeds::fromRobotRelativeSpeeds(speeds, pose.rotation));
}

KinematicState RuckigAlign::toKinematicState(const Pose2d &pose, const ChassisSpeeds &speeds, const ChassisSpeeds &accel)
{
    return KinematicState{
        processPoseForRuckig(pose), processChassisSpeeds(speeds, pose), processChassisSpeeds(accel, pose)};
}

void RuckigAlign::setTargetState(const KinematicState &state)
{
    std::array<double, 3> targetPosition = state.position;
    targetPosition[2] =
        MathUtil::closestTarget(input.current_position[2], MathUtil::normalizeAngle(targetPosition[2]));
    input.target_position = targetPosition;
    input.target_velocity = state.velocity;
    input.target_acceleration = state.acceleration;
}

void RuckigAlign::applyAlignState(const RuckigAlignState &state)
{
    setTargetState(state.kinematicState);
    maximizeConstraints(maxVelocity, getProcessedMaxAcceleration());
    input.max_jerk = getProcessedMaxJerk();
    if (state.alignMode == AlignMode::Position)
    {
        setPositionModeTolerance();
    }
    else
    {
        setVelocityModeTolerance();
    }
}

std::array<double, 3> RuckigAlign::getProcessedMaxAcceleration() const
{
    std::array<double, 3> processedMaxAcceleration = {};

    for (int i = 0; i < maxAcceleration.size(); i++)
    {
        processedMaxAcceleration[i] = (isDecelerating[i] == DeceleratingState::DECELERATING
                                           ? maxDeceleration[i]
                                           : maxAcceleration[i]);
    }

    return processedMaxAcceleration;
}

std::array<double, 3> RuckigAlign::getProcessedMaxJerk() const
{
    std::array<double, 3> processedMaxJerk = maxJerk;

    for (int i = 0; i < maxJerk.size(); i++)
    {
        if (isDecelerating[i] == DeceleratingState::DECELERATING)
        {
            processedMaxJerk[i] = maxDejerk[i];
        }
    }

    return processedMaxJerk;
}

void RuckigAlign::maximizeConstraints(const std::array<double, 3> &maxVelocity, const std::array<double, 3> &maxAcceleration)
{
    std::array<double, 3> maxVelocityOutput = maxVelocity;
    std::array<double, 3> maxAccelerationOutput = maxAcceleration;

    std::array<double, 3> currentVel = input.current_velocity;
    std::array<double, 3> currentAcc = input.current_acceleration;

    for (int i = 0; i < maxVelocity.size(); i++)
    {
        if (fabs(currentVel[i]) > fabs(maxVelocity[i]))
        {
            maxVelocityOutput[i] = fabs(currentVel[i]);
        }

        if (fabs(currentAcc[i]) > fabs(maxAcceleration[i]))
        {
            maxAccelerationOutput[i] = fabs(currentAcc[i]);
        }
    }

    input.max_velocity = maxVelocityOutput;
    input.max_acceleration = maxAccelerationOutput;
}