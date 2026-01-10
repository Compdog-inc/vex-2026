#ifndef __R_MATH_H__
#define __R_MATH_H__

#include "vex.h"
#include <array>
#include <limits>
#include <vector>
#include <optional>
#include <unordered_set>
#include <algorithm>

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

    template <typename T>
    inline constexpr T lerp(const T &start, const T &end, double t)
    {
        return start + (end - start) * t;
    }

    inline std::array<double, 3> multiplyM33AndV3(const std::array<double, 3 * 3> &matrix,
                                                  const std::array<double, 3> &vector)
    {
        return {matrix[0 * 3 + 0] * vector[0] + matrix[0 * 3 + 1] * vector[1] + matrix[0 * 3 + 2] * vector[2],
                matrix[1 * 3 + 0] * vector[0] + matrix[1 * 3 + 1] * vector[1] + matrix[1 * 3 + 2] * vector[2],
                matrix[2 * 3 + 0] * vector[0] + matrix[2 * 3 + 1] * vector[1] + matrix[2 * 3 + 2] * vector[2]};
    }

    template <typename T>
    const std::pair<double, T> *floorKey(
        const std::vector<std::pair<double, T>> &vec, double key)
    {
        auto it = std::lower_bound(vec.begin(), vec.end(), key,
                                   [](const auto &p, double val)
                                   { return p.first < val; });

        if (it == vec.begin() && it->first > key)
            return nullptr;
        if (it == vec.end() || it->first > key)
            --it;
        return &(*it);
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

    constexpr Twist2d operator*(double factor) const
    {
        return Twist2d{dx * factor, dy * factor, dtheta * factor};
    }
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

    Pose2d operator+(const Transform2d &other) const;
    Transform2d operator-(const Pose2d &other) const;
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

template <typename T>
class TimeInterpolatableBuffer
{
public:
    /**
     * Create a new TimeInterpolatableBuffer.
     *
     * @param historySize  The history size of the buffer.
     * @param func The function used to interpolate between values.
     */
    TimeInterpolatableBuffer(double historySize,
                             std::function<T(const T &, const T &, double)> func)
        : m_historySize(historySize), m_interpolatingFunc(func) {}

    /**
     * Create a new TimeInterpolatableBuffer. By default, the interpolation
     * function is lerp except for Pose2d, which uses the pose exponential.
     *
     * @param historySize  The history size of the buffer.
     */
    explicit TimeInterpolatableBuffer(double historySize)
        : m_historySize(historySize),
          m_interpolatingFunc([](const T &start, const T &end, double t)
                              { return MathUtil::lerp(start, end, t); }) {}

    /**
     * Add a sample to the buffer.
     *
     * @param time   The timestamp of the sample.
     * @param sample The sample object.
     */
    void addSample(double time, T sample)
    {
        // Add the new state into the vector
        if (m_pastSnapshots.size() == 0 || time > m_pastSnapshots.back().first)
        {
            m_pastSnapshots.emplace_back(time, sample);
        }
        else
        {
            auto first_after = std::upper_bound(
                m_pastSnapshots.begin(), m_pastSnapshots.end(), time,
                [](auto t, const auto &pair)
                { return t < pair.first; });

            if (first_after == m_pastSnapshots.begin())
            {
                // All entries come after the sample
                m_pastSnapshots.insert(first_after, std::pair{time, sample});
            }
            else if (auto last_not_greater_than = first_after - 1;
                     last_not_greater_than == m_pastSnapshots.begin() ||
                     last_not_greater_than->first < time)
            {
                // Some entries come before the sample, but none are recorded with the
                // same time
                m_pastSnapshots.insert(first_after, std::pair{time, sample});
            }
            else
            {
                // An entry exists with the same recorded time
                last_not_greater_than->second = sample;
            }
        }
        while (time - m_pastSnapshots[0].first > m_historySize)
        {
            m_pastSnapshots.erase(m_pastSnapshots.begin());
        }
    }

    /** Clear all old samples. */
    void clear() { m_pastSnapshots.clear(); }

    /**
     * Sample the buffer at the given time. If the buffer is empty, an empty
     * optional is returned.
     *
     * @param time The time at which to sample the buffer.
     */
    bool sample(double time, T *result) const
    {
        if (m_pastSnapshots.empty())
        {
            return false;
        }

        // We will perform a binary search to find the index of the element in the
        // vector that has a timestamp that is equal to or greater than the vision
        // measurement timestamp.

        if (time <= m_pastSnapshots.front().first)
        {
            *result = m_pastSnapshots.front().second;
            return true;
        }
        if (time > m_pastSnapshots.back().first)
        {
            *result = m_pastSnapshots.back().second;
            return true;
        }
        if (m_pastSnapshots.size() < 2)
        {
            *result = m_pastSnapshots[0].second;
            return true;
        }

        // Get the iterator which has a key no less than the requested key.
        auto upper_bound = std::lower_bound(
            m_pastSnapshots.begin(), m_pastSnapshots.end(), time,
            [](const auto &pair, auto t)
            { return t > pair.first; });

        if (upper_bound == m_pastSnapshots.begin())
        {
            *result = upper_bound->second;
            return true;
        }

        auto lower_bound = upper_bound - 1;

        double t = ((time - lower_bound->first) /
                    (upper_bound->first - lower_bound->first));

        *result = m_interpolatingFunc(lower_bound->second, upper_bound->second, t);
        return true;
    }

    /**
     * Grant access to the internal sample buffer. Used in Pose Estimation to
     * replay odometry inputs stored within this buffer.
     */
    std::vector<std::pair<double, T>> &getInternalBuffer()
    {
        return m_pastSnapshots;
    }

    /**
     * Grant access to the internal sample buffer.
     */
    const std::vector<std::pair<double, T>> &getInternalBuffer() const
    {
        return m_pastSnapshots;
    }

private:
    double m_historySize;
    std::vector<std::pair<double, T>> m_pastSnapshots;
    std::function<T(const T &, const T &, double)> m_interpolatingFunc;
};

// Template specialization to ensure that Pose2d uses pose exponential
template <>
inline TimeInterpolatableBuffer<Pose2d>::TimeInterpolatableBuffer(
    double historySize)
    : m_historySize(historySize),
      m_interpolatingFunc([](const Pose2d &start, const Pose2d &end, double t)
                          {
        if (t < 0) {
          return start;
        } else if (t >= 1) {
          return end;
        } else {
          Twist2d twist = start.log(end);
          Twist2d scaledTwist = twist * t;
          return start.exp(scaledTwist);
        } }) {}
#endif