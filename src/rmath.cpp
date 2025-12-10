#include "rmath.h"

Rotation2d::Rotation2d(double x, double y)
{
    double magnitude = hypot(x, y);
    if (magnitude > 1e-6)
    {
        cosAngle = x / magnitude;
        sinAngle = y / magnitude;
    }
    else
    {
        cosAngle = 1.0;
        sinAngle = 0.0;
    }
    value = atan2(sinAngle, cosAngle);
}

Rotation2d::Rotation2d(double angle)
    : value(angle)
{
    cosAngle = cos(angle);
    sinAngle = sin(angle);
}

Rotation2d Rotation2d::rotateBy(const Rotation2d &other) const
{
    return Rotation2d{
        cosAngle * other.cosAngle - sinAngle * other.sinAngle,
        cosAngle * other.sinAngle + sinAngle * other.cosAngle};
}

Rotation2d Rotation2d::operator+(const Rotation2d &other) const
{
    return rotateBy(other);
}

Rotation2d Rotation2d::operator-(const Rotation2d &other) const
{
    return rotateBy(-other);
}

Rotation2d Rotation2d::operator-() const
{
    return Rotation2d{-value};
}

Translation2d Translation2d::rotateBy(const Rotation2d &rotation) const
{
    return Translation2d{
        x * rotation.cosAngle - y * rotation.sinAngle,
        x * rotation.sinAngle + y * rotation.cosAngle};
}

Translation2d Translation2d::operator+(const Translation2d &other) const
{
    return Translation2d{x + other.x, y + other.y};
}

Translation2d Translation2d::operator-(const Translation2d &other) const
{
    return Translation2d{x - other.x, y - other.y};
}

Translation2d Translation2d::operator*(double scalar) const
{
    return Translation2d{x * scalar, y * scalar};
}

Transform2d::Transform2d(Translation2d translation, Rotation2d rotation)
    : translation(translation), rotation(rotation)
{
}

Transform2d::Transform2d(const Pose2d &initial, const Pose2d &last)
    : translation((last.translation - initial.translation).rotateBy(-initial.rotation)),
      rotation(last.rotation - initial.rotation)
{
}

Pose2d Pose2d::exp(const Twist2d &twist) const
{
    double dx = twist.dx;
    double dy = twist.dy;
    double dtheta = twist.dtheta;

    double sinTheta = sin(dtheta);
    double cosTheta = cos(dtheta);

    double s;
    double c;
    if (fabs(dtheta) < 1E-9)
    {
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
        c = 0.5 * dtheta;
    }
    else
    {
        s = sinTheta / dtheta;
        c = (1 - cosTheta) / dtheta;
    }

    Transform2d transform =
        Transform2d{
            Translation2d{dx * s - dy * c, dx * c + dy * s},
            Rotation2d{cosTheta, sinTheta}};

    return transformBy(transform);
}

Twist2d Pose2d::log(const Pose2d &end) const
{
    Pose2d transform = end.relativeTo(*this);
    double dtheta = transform.rotation.value;
    double halfDtheta = dtheta / 2.0;

    double cosMinusOne = transform.rotation.cosAngle - 1;

    double halfThetaByTanOfHalfDtheta;
    if (fabs(cosMinusOne) < 1E-9)
    {
        halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    }
    else
    {
        halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.rotation.sinAngle) / cosMinusOne;
    }

    Translation2d translationPart =
        transform
            .translation
            .rotateBy(Rotation2d{halfThetaByTanOfHalfDtheta, -halfDtheta}) *
        hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

    return Twist2d{translationPart.x, translationPart.y, dtheta};
}

Pose2d Pose2d::transformBy(const Transform2d &other) const
{
    return Pose2d{
        translation + other.translation.rotateBy(rotation),
        other.rotation + rotation};
}

Pose2d Pose2d::relativeTo(const Pose2d &other) const
{
    Transform2d transform = Transform2d{other, *this};
    return Pose2d{transform.translation, transform.rotation};
}

Pose2d Pose2d::operator+(const Transform2d &other) const
{
    return transformBy(other);
}

Transform2d Pose2d::operator-(const Pose2d &other) const
{
    Pose2d pose = this->relativeTo(other);
    return Transform2d{pose.translation, pose.rotation};
}

const Pose2d Pose2d::kZero = {Translation2d{0.0, 0.0}, Rotation2d{0.0}};

/**
 * Converts field-relative speeds to robot-relative speeds.
 * @param fieldRelative The desired field-relative speeds.
 * @param robotAngle The current robot angle in radians.
 * @return The robot-relative speeds.
 */
ChassisSpeeds ChassisSpeeds::fromFieldRelativeSpeeds(const ChassisSpeeds &fieldRelative, Rotation2d robotAngle)
{
    ChassisSpeeds robotRelative;
    robotRelative.vx = fieldRelative.vx * robotAngle.cosAngle + fieldRelative.vy * robotAngle.sinAngle;
    robotRelative.vy = -fieldRelative.vx * robotAngle.sinAngle + fieldRelative.vy * robotAngle.cosAngle;
    robotRelative.omega = fieldRelative.omega;
    return robotRelative;
}

/**
 * Converts robot-relative speeds to field-relative speeds.
 * @param robotRelative The desired robot-relative speeds.
 * @param robotAngle The current robot angle in radians.
 * @return The field-relative speeds.
 */
ChassisSpeeds ChassisSpeeds::fromRobotRelativeSpeeds(const ChassisSpeeds &robotRelative, Rotation2d robotAngle)
{
    ChassisSpeeds fieldRelative;
    fieldRelative.vx = robotRelative.vx * robotAngle.cosAngle - robotRelative.vy * robotAngle.sinAngle;
    fieldRelative.vy = robotRelative.vx * robotAngle.sinAngle + robotRelative.vy * robotAngle.cosAngle;
    fieldRelative.omega = robotRelative.omega;
    return fieldRelative;
}

ChassisSpeeds ChassisSpeeds::discretize(const ChassisSpeeds &speeds, double deltaTime)
{
    // Construct the desired pose after a timestep, relative to the current pose. The desired pose
    // has decoupled translation and rotation.
    Pose2d desiredDeltaPose =
        Pose2d{
            Translation2d{speeds.vx * deltaTime,
                          speeds.vy * deltaTime},
            Rotation2d{speeds.omega * deltaTime}};

    // Find the chassis translation/rotation deltas in the robot frame that move the robot from its
    // current pose to the desired pose
    Twist2d twist = Pose2d::kZero.log(desiredDeltaPose);

    // Turn the chassis translation/rotation deltas into average velocities
    return ChassisSpeeds{twist.dx / deltaTime, twist.dy / deltaTime, twist.dtheta / deltaTime};
}

PIDController::PIDController(double kP, double kI, double kD, double period)
    : kP(kP), kI(kI), kD(kD), period(period), prevError(0.0), totalError(0.0), haveMeasurement(false), haveSetpoint(false)
{
}

void PIDController::setPID(double p, double i, double d)
{
    kP = p;
    kI = i;
    kD = d;
}

void PIDController::setP(double p)
{
    kP = p;
}

void PIDController::setI(double i)
{
    kI = i;
}

void PIDController::setD(double d)
{
    kD = d;
}

void PIDController::setIZone(double iZone)
{
    this->iZone = iZone;
}

double PIDController::getP() const
{
    return kP;
}

double PIDController::getI() const
{
    return kI;
}

double PIDController::getD() const
{
    return kD;
}

double PIDController::getIZone() const
{
    return iZone;
}

double PIDController::getPeriod() const
{
    return period;
}

double PIDController::getErrorTolerance() const
{
    return errorTolerance;
}

double PIDController::getErrorDerivativeTolerance() const
{
    return errorDerivativeTolerance;
}

double PIDController::getAccumulatedError() const
{
    return totalError;
}

void PIDController::setSetpoint(double setpoint)
{
    this->setpoint = setpoint;
    haveSetpoint = true;

    if (continuous)
    {
        double errorBound = (maximumInput - minimumInput) / 2.0;
        error = MathUtil::inputModulus(setpoint - measurement, -errorBound, errorBound);
    }
    else
    {
        error = setpoint - measurement;
    }

    errorDerivative = (error - prevError) / period;
}

double PIDController::getSetpoint() const
{
    return setpoint;
}

bool PIDController::atSetpoint() const
{
    return haveMeasurement && haveSetpoint && fabs(error) < errorTolerance && fabs(errorDerivative) < errorDerivativeTolerance;
}

void PIDController::enableContinuousInput(double minimumInput, double maximumInput)
{
    continuous = true;
    this->minimumInput = minimumInput;
    this->maximumInput = maximumInput;
}

void PIDController::disableContinuousInput()
{
    continuous = false;
}

bool PIDController::isContinuousInputEnabled() const
{
    return continuous;
}

void PIDController::setIntegratorRange(double minimumIntegral, double maximumIntegral)
{
    this->minimumIntegral = minimumIntegral;
    this->maximumIntegral = maximumIntegral;
}

void PIDController::setTolerance(double errorTolerance, double errorDerivativeTolerance)
{
    this->errorTolerance = errorTolerance;
    this->errorDerivativeTolerance = errorDerivativeTolerance;
}

double PIDController::getError() const
{
    return error;
}

double PIDController::getErrorDerivative() const
{
    return errorDerivative;
}

double PIDController::calculate(double measurement, double setpoint)
{
    this->setpoint = setpoint;
    haveSetpoint = true;
    return calculate(measurement);
}

double PIDController::calculate(double measurement)
{
    this->measurement = measurement;
    prevError = error;
    haveMeasurement = true;

    // Calculate error
    if (continuous)
    {
        double errorBound = (maximumInput - minimumInput) / 2.0;
        error = MathUtil::inputModulus(setpoint - measurement, -errorBound, errorBound);
    }
    else
    {
        error = setpoint - measurement;
    }

    // Calculate derivative of error
    errorDerivative = (error - prevError) / period;

    // If the absolute value of the position error is greater than IZone, reset the total error
    if (fabs(error) > iZone)
    {
        totalError = 0;
    }
    else if (kI != 0)
    {
        totalError =
            MathUtil::clamp(
                totalError + error * period,
                minimumIntegral / kI,
                maximumIntegral / kI);
    }

    return kP * error + kI * totalError + kD * errorDerivative;
}

void PIDController::reset()
{
    error = 0.0;
    prevError = 0.0;
    totalError = 0.0;
    errorDerivative = 0;
    haveMeasurement = false;
}