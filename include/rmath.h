#ifndef __R_MATH_H__
#define __R_MATH_H__

#include "vex.h"
#include <array>
#include <limits>
#include <optional>
#include <unordered_set>

namespace MathUtil
{
    /**
     * Returns modulus of input.
     *
     * @param input Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    inline double inputModulus(double input, double minimumInput, double maximumInput)
    {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int)((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int)((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    inline double clamp(double value, double low, double high)
    {
        return fmax(low, fmin(value, high));
    }

    /**
     *  Normalize an angle to [-PI, PI] (for use with {@link #closestTarget})
     */
    inline double normalizeAngle(double angle)
    {
        angle = fmod(fmod(angle + M_PI, M_TWOPI) + M_TWOPI, M_TWOPI) - M_PI;
        return angle;
    }

    /**
     * Returns the closest unbounded axis position corresponding to the target angle.
     *
     * @param currentPos Current unbounded axis position (radians)
     * @param targetAngle Target angle (radians, in [-PI, PI])
     * @return Closest unbounded position to the target angle
     */
    inline double closestTarget(double currentPos, double targetAngle)
    {
        // Compute the nearest multiple of 2pi
        double k = round((currentPos - targetAngle) / M_TWOPI);
        // Return the unbounded target position
        return targetAngle + k * M_TWOPI;
    }

    template <typename T>
    inline constexpr int signum(T x, std::false_type is_signed)
    {
        return T(0) < x;
    }

    template <typename T>
    inline constexpr int signum(T x, std::true_type is_signed)
    {
        return (T(0) < x) - (x < T(0));
    }

    template <typename T>
    inline constexpr int signum(T x)
    {
        return signum(x, std::is_signed<T>());
    }

    inline constexpr bool signEq(double a, double b)
    {
        return signum(a) == signum(b);
    }

    inline constexpr double toRadians(double degrees)
    {
        return degrees * (M_PI / 180.0);
    }

    template <typename T>
    bool intersects(const std::unordered_set<T> &a,
                    const std::unordered_set<T> &b)
    {
        const auto &smaller = (a.size() < b.size()) ? a : b;
        const auto &larger = (a.size() < b.size()) ? b : a;

        for (const auto &x : smaller)
            if (larger.count(x))
                return true;

        return false;
    }
}; // namespace MathUtil

typedef struct Rotation2d
{
    double value;
    double cosAngle;
    double sinAngle;

    Rotation2d(double x, double y);
    Rotation2d(double angle);

    Rotation2d rotateBy(const Rotation2d &other) const;

    Rotation2d operator+(const Rotation2d &other) const;
    Rotation2d operator-(const Rotation2d &other) const;
    Rotation2d operator-() const;

} Rotation2d;

typedef struct Translation2d
{
    double x;
    double y;

    Translation2d rotateBy(const Rotation2d &rotation) const;

    Translation2d operator+(const Translation2d &other) const;
    Translation2d operator-(const Translation2d &other) const;
    Translation2d operator*(double scalar) const;
} Translation2d;

typedef struct Pose2d Pose2d;

typedef struct Transform2d
{
    Translation2d translation;
    Rotation2d rotation;

    Transform2d(const Pose2d &initial, const Pose2d &last);
    Transform2d(Translation2d translation, Rotation2d rotation);
} Transform2d;

typedef struct Twist2d
{
    double dx;
    double dy;
    double dtheta; // in radians
} Twist2d;

typedef struct Pose2d
{
    Translation2d translation;
    Rotation2d rotation;

    static const Pose2d kZero;

    Pose2d exp(const Twist2d &twist) const;
    Twist2d log(const Pose2d &end) const;

    Pose2d transformBy(const Transform2d &other) const;
    Pose2d relativeTo(const Pose2d &other) const;
} Pose2d;

typedef struct ChassisSpeeds
{
    double vx;    // forward velocity (meters per second)
    double vy;    // sideways velocity (meters per second)
    double omega; // angular velocity (radians per second)

    /**
     * Converts field-relative speeds to robot-relative speeds.
     * @param fieldRelative The desired field-relative speeds.
     * @param robotAngle The current robot angle in radians.
     * @return The robot-relative speeds.
     */
    static ChassisSpeeds fromFieldRelativeSpeeds(const ChassisSpeeds &fieldRelative, Rotation2d robotAngle);

    /**
     * Converts robot-relative speeds to field-relative speeds.
     * @param robotRelative The desired robot-relative speeds.
     * @param robotAngle The current robot angle in radians.
     * @return The field-relative speeds.
     */
    static ChassisSpeeds fromRobotRelativeSpeeds(const ChassisSpeeds &robotRelative, Rotation2d robotAngle);

    static ChassisSpeeds discretize(const ChassisSpeeds &speeds, double deltaTime);
} ChassisSpeeds;

class PIDController
{
public:
    PIDController(double kP, double kI, double kD, double period = 0.02);

    void setPID(double p, double i, double d);

    void setP(double p);
    void setI(double i);
    void setD(double d);

    void setIZone(double iZone);

    double getP() const;
    double getI() const;
    double getD() const;

    double getIZone() const;

    double getPeriod() const;

    double getErrorTolerance() const;

    double getErrorDerivativeTolerance() const;

    double getAccumulatedError() const;

    void setSetpoint(double setpoint);

    double getSetpoint() const;

    bool atSetpoint() const;

    void enableContinuousInput(double minimumInput, double maximumInput);

    void disableContinuousInput();

    bool isContinuousInputEnabled() const;

    void setIntegratorRange(double minimumIntegral, double maximumIntegral);

    void setTolerance(double errorTolerance, double errorDerivativeTolerance = std::numeric_limits<double>::infinity());

    double getError() const;
    double getErrorDerivative() const;

    double calculate(double measurement, double setpoint);
    double calculate(double measurement);
    void reset();

private:
    double kP;
    double kI;
    double kD;

    double iZone;

    double period;

    double maximumIntegral;
    double minimumIntegral;
    double maximumInput;
    double minimumInput;

    bool continuous;

    double error;
    double errorDerivative;
    double prevError;
    double totalError;
    double errorTolerance = 0.05;
    double errorDerivativeTolerance = std::numeric_limits<double>::infinity();

    double setpoint;
    double measurement;

    bool haveMeasurement;
    bool haveSetpoint;
};
#endif