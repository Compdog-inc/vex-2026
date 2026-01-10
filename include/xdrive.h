#ifndef __X_DRIVE_H__
#define __X_DRIVE_H__

#include "vex.h"
#include <array>
#include <cmath>
#include <algorithm>

#include "rmath.h"

typedef struct XDriveModuleState
{
    double velocity;
    double angle; // in radians
} XDriveModuleState;

typedef struct XDriveModulePosition
{
    double distance; // meters
    double angle;    // in radians
} XDriveModulePosition;

typedef struct XDriveModule
{
    Translation2d position; // relative to robot center
    Rotation2d rotation;
} XDriveModule;

typedef struct XDriveModuleData
{
    double fx;
    double fy;
    double sc;
} XDriveModuleData;

template <std::size_t _Nm>
class XDriveKinematics
{
public:
    XDriveKinematics(std::array<XDriveModule, _Nm> modules)
        : modules(modules)
    {
        for (size_t i = 0; i < modules.size(); ++i)
        {
            moduleData[i] = calculateModuleData(modules[i]);
        }

        setupMatrix();
    }

    void setRotationOrigin(const Translation2d &origin)
    {
        rotationOrigin = origin;
    }

    Translation2d getRotationOrigin() const
    {
        return rotationOrigin;
    }

    void setRobotOrigin(const Translation2d &origin)
    {
        robotOrigin = origin;

        for (size_t i = 0; i < modules.size(); ++i)
        {
            moduleData[i] = calculateModuleData(modules[i]);
        }

        setupMatrix();
    }

    std::array<XDriveModuleState, _Nm> toModuleStates(const ChassisSpeeds &chassisSpeeds) const
    {
        std::array<XDriveModuleState, _Nm> moduleStates{};

        for (size_t i = 0; i < moduleStates.size(); ++i)
        {
            Translation2d rel = modules[i].position - rotationOrigin;

            moduleStates[i].velocity = moduleData[i].fx * (chassisSpeeds.vx - rel.y * chassisSpeeds.omega) +
                                       moduleData[i].fy * (chassisSpeeds.vy + rel.x * chassisSpeeds.omega);
            moduleStates[i]
                .angle = modules[i].rotation.value;
        }

        return moduleStates;
    }

    ChassisSpeeds toChassisSpeeds(const std::array<XDriveModuleState, _Nm> &moduleStates) const
    {
        double b1 = 0.0;
        double b2 = 0.0;
        double b3 = 0.0;

        for (size_t i = 0; i < moduleStates.size(); ++i)
        {
            b1 += moduleData[i].fx * moduleStates[i].velocity;
            b2 += moduleData[i].fy * moduleStates[i].velocity;
            b3 += moduleData[i].sc * moduleStates[i].velocity;
        }

        double bp1 = b2 * m_a33 - m_a23 * b3;
        double bp2 = b2 * m_a23 - b3 * m_a22;
        double bp3 = b3 * m_a12 - b2 * m_a13;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds.vx = (b1 * p_1 -
                            m_a12 * bp1 +
                            m_a13 * bp2) /
                           m_det;

        chassisSpeeds.vy = (m_a11 * bp1 -
                            b1 * p_2 +
                            m_a13 * bp3) /
                           m_det;

        chassisSpeeds.omega = (m_a11 * (-bp2) -
                               m_a12 * bp3 +
                               b1 * p_3) /
                              m_det;

        return chassisSpeeds;
    }

    Twist2d toTwist2d(const std::array<XDriveModulePosition, _Nm> &moduleDeltas) const
    {
        std::array<XDriveModuleState, _Nm> moduleStates{};
        for (size_t i = 0; i < moduleDeltas.size(); ++i)
        {
            moduleStates[i].velocity = moduleDeltas[i].distance;
            moduleStates[i].angle = moduleDeltas[i].angle;
        }

        ChassisSpeeds speeds = toChassisSpeeds(moduleStates);
        Twist2d twist;
        twist.dx = speeds.vx;
        twist.dy = speeds.vy;
        twist.dtheta = speeds.omega;
        return twist;
    }

    Twist2d toTwist2d(const std::array<XDriveModulePosition, _Nm> &lastModulePositions,
                      const std::array<XDriveModulePosition, _Nm> &currentModulePositions) const
    {
        std::array<XDriveModulePosition, _Nm> deltaPositions{};

        for (size_t i = 0; i < deltaPositions.size(); ++i)
        {
            double deltaDistance = currentModulePositions[i].distance - lastModulePositions[i].distance;
            deltaPositions[i].distance = deltaDistance;
            deltaPositions[i].angle = currentModulePositions[i].angle;
        }

        return toTwist2d(deltaPositions);
    }

    static std::array<XDriveModuleState, _Nm> desaturateWheelSpeeds(const std::array<XDriveModuleState, _Nm> &moduleStates, double attainableMaxSpeedMetersPerSecond)
    {
        double maxFoundVelocity = 0.0;
        for (size_t i = 0; i < moduleStates.size(); ++i)
        {
            if (fabs(moduleStates[i].velocity) > maxFoundVelocity)
            {
                maxFoundVelocity = fabs(moduleStates[i].velocity);
            }
        }

        if (maxFoundVelocity > attainableMaxSpeedMetersPerSecond)
        {
            std::array<XDriveModuleState, _Nm> normalizedStates{};
            double scale = attainableMaxSpeedMetersPerSecond / maxFoundVelocity;
            for (size_t i = 0; i < moduleStates.size(); ++i)
            {
                normalizedStates[i].velocity = moduleStates[i].velocity * scale;
                normalizedStates[i].angle = moduleStates[i].angle;
            }
            return normalizedStates;
        }
        else
        {
            return moduleStates;
        }
    }

private:
    std::array<XDriveModule, _Nm> modules;
    std::array<XDriveModuleData, _Nm> moduleData;

    Translation2d rotationOrigin{0.0, 0.0};
    Translation2d robotOrigin{0.0, 0.0};

    double m_a11;
    double m_a12;
    double m_a13;
    double m_a22;
    double m_a23;
    double m_a33;
    double p_1;
    double p_2;
    double p_3;
    double m_det;

    XDriveModuleData calculateModuleData(const XDriveModule &module) const
    {
        Translation2d rel = module.position - robotOrigin;
        XDriveModuleData data;
        data.fx = module.rotation.cosAngle;
        data.fy = module.rotation.sinAngle;
        data.sc = -rel.y * data.fx + rel.x * data.fy;
        return data;
    }

    void setupMatrix()
    {
        m_a11 = 0.0;
        m_a12 = 0.0;
        m_a13 = 0.0;
        m_a22 = 0.0;
        m_a23 = 0.0;
        m_a33 = 0.0;

        for (size_t i = 0; i < modules.size(); ++i)
        {
            m_a11 += moduleData[i].fx * moduleData[i].fx;
            m_a12 += moduleData[i].fx * moduleData[i].fy;
            m_a13 += moduleData[i].fx * moduleData[i].sc;
            m_a22 += moduleData[i].fy * moduleData[i].fy;
            m_a23 += moduleData[i].fy * moduleData[i].sc;
            m_a33 += moduleData[i].sc * moduleData[i].sc;
        }

        p_1 = m_a22 * m_a33 - m_a23 * m_a23;
        p_2 = m_a12 * m_a33 - m_a13 * m_a23;
        p_3 = m_a12 * m_a23 - m_a13 * m_a22;

        m_det = m_a11 * p_1 -
                m_a12 * p_2 +
                m_a13 * p_3;
    }
};

template <std::size_t _Nm>
class XDriveOdometry
{
public:
    XDriveOdometry(XDriveKinematics<_Nm> &kinematics, const Rotation2d &gyroAngle, const std::array<XDriveModulePosition, _Nm> &modulePositions, const Pose2d &initialPose)
        : kinematics(kinematics),
          lastModulePositions(modulePositions),
          robotPose(initialPose),
          gyroOffset(initialPose.rotation - gyroAngle),
          previousAngle(initialPose.rotation)
    {
    }

    Pose2d update(const Rotation2d &gyroAngle, const std::array<XDriveModulePosition, _Nm> &modulePositions)
    {
        Rotation2d angle = gyroAngle + gyroOffset;

        Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
        twist.dtheta = angle.value - previousAngle.value;

        lastModulePositions = modulePositions;
        previousAngle = angle;

        Pose2d newPose = robotPose.exp(twist);
        robotPose = {newPose.translation, angle};

        return robotPose;
    }

    Pose2d getRobotPose() const
    {
        return robotPose;
    }

    void resetPose(const Pose2d &newPose)
    {
        gyroOffset = gyroOffset + (newPose.rotation - robotPose.rotation);
        robotPose = newPose;
        previousAngle = newPose.rotation;
    }

private:
    XDriveKinematics<_Nm> kinematics;
    std::array<XDriveModulePosition, _Nm> lastModulePositions;
    Pose2d robotPose;
    Rotation2d gyroOffset;
    Rotation2d previousAngle;
};

template <std::size_t _Nm>
class XDrivePoseEstimator
{
private:
    typedef struct VisionUpdate
    {
        Pose2d visionPose;
        Pose2d odometryPose;

        VisionUpdate(const Pose2d &visionPose, const Pose2d &odometryPose)
            : visionPose(visionPose), odometryPose(odometryPose)
        {
        }

        Pose2d compensate(const Pose2d &pose) const
        {
            Transform2d delta = pose - odometryPose;
            return visionPose + delta;
        }
    } VisionUpdate;

public:
    XDrivePoseEstimator(
        XDriveKinematics<_Nm> &kinematics,
        const Rotation2d &gyroAngle,
        const std::array<XDriveModulePosition, _Nm> &modulePositions,
        const Pose2d &initialPose)
        : XDrivePoseEstimator(
              kinematics,
              gyroAngle,
              modulePositions,
              initialPose,
              {0.1, 0.1, 0.1},
              {0.9, 0.9, 0.9})
    {
    }

    XDrivePoseEstimator(
        XDriveKinematics<_Nm> &kinematics,
        const Rotation2d &gyroAngle,
        const std::array<XDriveModulePosition, _Nm> &modulePositions,
        const Pose2d &initialPose,
        const std::array<double, 3> &stateStdDevs,
        const std::array<double, 3> &visionMeasurementStdDevs)
        : XDrivePoseEstimator(kinematics,
                              XDriveOdometry<_Nm>(kinematics, gyroAngle, modulePositions, initialPose),
                              stateStdDevs,
                              visionMeasurementStdDevs)
    {
    }

    void setVisionMeasurementStdDevs(const std::array<double, 3> &stdDevs)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            double r = stdDevs[i] * stdDevs[i];
            if (q[i] == 0.0)
            {
                visionK[i * 3 + i] = 0.0;
            }
            else
            {
                visionK[i * 3 + i] = q[i] / (q[i] + std::sqrt(q[i] * r));
            }

            for (size_t j = 0; j < 3; ++j)
            {
                if (j != i)
                {
                    visionK[i * 3 + j] = 0.0;
                }
            }
        }
    }

    void resetPose(const Pose2d &pose)
    {
        odometry.resetPose(pose);
        odometryPoseBuffer.clear();
        visionUpdates.clear();
        poseEstimate = odometry.getRobotPose();
    }

    Pose2d getEstimatedPosition() const
    {
        return poseEstimate;
    }

    bool sampleAt(double timestampSeconds, Pose2d *outPose) const
    {
        if (odometryPoseBuffer.getInternalBuffer().empty())
        {
            return false;
        }

        // Step 1: Make sure timestamp matches the sample from the odometry pose buffer. (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().front().first;
        double newestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().back().first;
        timestampSeconds =
            MathUtil::clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        // Step 2: If there are no applicable vision updates, use the odometry-only information.
        if (visionUpdates.empty() || timestampSeconds < visionUpdates.front().first)
        {
            return odometryPoseBuffer.sample(timestampSeconds, outPose);
        }

        // Step 3: Get the latest vision update from before or at the timestamp to sample at.
        auto floorKeyPtr = MathUtil::floorKey(visionUpdates, timestampSeconds);
        VisionUpdate visionUpdate = floorKeyPtr ? floorKeyPtr->second : visionUpdates.front().second;

        // Step 4: Get the pose measured by odometry at the time of the sample.
        Pose2d odometryEstimate = Pose2d::kZero;
        bool success = odometryPoseBuffer.sample(timestampSeconds, &odometryEstimate);

        if (!success)
        {
            return false;
        }

        *outPose = visionUpdate.compensate(odometryEstimate);
        return true;
    }

    void addVisionMeasurement(const Pose2d &visionPose, double timestampSeconds)
    {
        // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
        if (odometryPoseBuffer.getInternalBuffer().empty() || odometryPoseBuffer.getInternalBuffer().back().first - kBufferDuration > timestampSeconds)
        {
            return;
        }

        // Step 1: Clean up any old entries
        cleanUpVisionUpdates();

        // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
        Pose2d odometrySample = Pose2d::kZero;
        bool success = odometryPoseBuffer.sample(timestampSeconds, &odometrySample);

        if (!success)
        {
            return;
        }

        // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
        // made.
        Pose2d visionSample = Pose2d::kZero;
        success = sampleAt(timestampSeconds, &visionSample);

        if (!success)
        {
            return;
        }

        // Step 4: Measure the transform between the old pose estimate and the vision pose.
        Transform2d transform = visionPose - visionSample;

        // Step 5: We should not trust the transform entirely, so instead we scale this transform by a
        // Kalman
        // gain matrix representing how much we trust vision measurements compared to our current pose.
        auto k_times_transform =
            MathUtil::multiplyM33AndV3(visionK, {transform.translation.x, transform.translation.y, transform.rotation.value});

        // Step 6: Convert back to Transform2d.
        Transform2d scaledTransform{
            Transform2d{
                Translation2d{k_times_transform[0],
                              k_times_transform[1]},
                Rotation2d{k_times_transform[2]}}};

        // Step 7: Calculate and record the vision update.
        VisionUpdate visionUpdate{visionSample + scaledTransform, odometrySample};
        putVisionMeasurement(timestampSeconds, visionUpdate);

        // Step 8: Remove later vision measurements. (Matches previous behavior)
        auto first_later = std::upper_bound(
            visionUpdates.begin(), visionUpdates.end(), timestampSeconds,
            [](auto t, const auto &pair)
            { return t < pair.first; });
        visionUpdates.erase(first_later, visionUpdates.end());

        // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
        // it's guaranteed to be the latest vision update.
        poseEstimate = visionUpdate.compensate(odometry.getRobotPose());
    }

    void addVisionMeasurement(const Pose2d &visionPose, double timestampSeconds, const std::array<double, 3> &visionMeasurementStdDevs)
    {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionPose, timestampSeconds);
    }

    Pose2d update(const Rotation2d &gyroAngle, const std::array<XDriveModulePosition, _Nm> &modulePositions)
    {
        return updateWithTime(vex::timer::systemHighResolution() / 1000.0, gyroAngle, modulePositions);
    }

    Pose2d updateWithTime(double currentTimeSeconds, const Rotation2d &gyroAngle, const std::array<XDriveModulePosition, _Nm> &modulePositions)
    {
        Pose2d odometryEstimate = odometry.update(gyroAngle, modulePositions);

        odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate);

        if (visionUpdates.empty())
        {
            poseEstimate = odometryEstimate;
        }
        else
        {
            VisionUpdate visionUpdate = visionUpdates.back().second;
            poseEstimate = visionUpdate.compensate(odometryEstimate);
        }

        return getEstimatedPosition();
    }

private:
    XDrivePoseEstimator(
        XDriveKinematics<_Nm> &kinematics,
        const XDriveOdometry<_Nm> &odometry,
        const std::array<double, 3> &stateStdDevs,
        const std::array<double, 3> &visionMeasurementStdDevs)
        : odometry(odometry),
          poseEstimate(odometry.getRobotPose())
    {
        for (size_t i = 0; i < 3; ++i)
        {
            q[i] = stateStdDevs[i] * stateStdDevs[i];
        }

        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    void putVisionMeasurement(double timestamp, const VisionUpdate &update)
    {
        if (visionUpdates.size() == 0 || timestamp > visionUpdates.back().first)
        {
            visionUpdates.emplace_back(timestamp, update);
        }
        else
        {
            auto first_after = std::upper_bound(
                visionUpdates.begin(), visionUpdates.end(), timestamp,
                [](auto t, const auto &pair)
                { return t < pair.first; });

            if (first_after == visionUpdates.begin())
            {
                // All entries come after the sample
                visionUpdates.insert(first_after, std::pair{timestamp, update});
            }
            else if (auto last_not_greater_than = first_after - 1;
                     last_not_greater_than == visionUpdates.begin() ||
                     last_not_greater_than->first < timestamp)
            {
                // Some entries come before the sample, but none are recorded with the
                // same time
                visionUpdates.insert(first_after, std::pair{timestamp, update});
            }
            else
            {
                // An entry exists with the same recorded time
                last_not_greater_than->second = update;
            }
        }
    }

    void cleanUpVisionUpdates()
    {
        if (odometryPoseBuffer.getInternalBuffer().empty())
        {
            return;
        }

        // Step 1: Find the oldest timestamp that needs a vision update.
        double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().front().first;

        // Step 2: If there are no vision updates before that timestamp, skip.
        if (visionUpdates.empty() || oldestOdometryTimestamp < visionUpdates.front().first)
        {
            return;
        }

        // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
        auto floorKeyPtr = MathUtil::floorKey(visionUpdates, oldestOdometryTimestamp);
        double newestNeededVisionUpdateTimestamp = floorKeyPtr ? floorKeyPtr->first : visionUpdates.front().first;

        // Step 4: Remove all entries strictly before the newest timestamp we need.
        auto first_needed = std::lower_bound(
            visionUpdates.begin(), visionUpdates.end(), newestNeededVisionUpdateTimestamp,
            [](const auto &pair, auto t)
            { return pair.first < t; });
        visionUpdates.erase(visionUpdates.begin(), first_needed);
    }

    XDriveOdometry<_Nm> odometry;
    Pose2d poseEstimate;
    std::array<double, 3> q;
    std::array<double, 3 * 3> visionK;

    constexpr static double kBufferDuration = 1.5;
    TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer{kBufferDuration};
    std::vector<std::pair<double, VisionUpdate>> visionUpdates;
};

template <std::size_t _Nm>
class XDrive
{
public:
    XDrive(double wheelDiameter,
           double attainableMaxSpeedMetersPerSecond,
           std::array<vex::motor, _Nm> motors,
           std::array<XDriveModule, _Nm> modules,
           vex::inertial &gyro,
           const Pose2d &initialPose)
        : motors(motors),
          modules(modules),
          gyro(gyro),
          lastVelocities(),
          accelerations(),
          wheelDiameter(wheelDiameter),
          wheelCircumference(wheelDiameter * M_PI),
          attainableMaxSpeedMetersPerSecond(attainableMaxSpeedMetersPerSecond),
          kinematics(modules),
          poseEstimator(kinematics,
                        getGyroRotation(),
                        getModulePositions(),
                        initialPose)
    {
    }

    void drive(ChassisSpeeds robotRelativeSpeeds, double deltaTime)
    {
        std::array<XDriveModuleState, _Nm> moduleStates = kinematics.toModuleStates(robotRelativeSpeeds);
        moduleStates = XDriveKinematics<_Nm>::desaturateWheelSpeeds(moduleStates, attainableMaxSpeedMetersPerSecond);
        ChassisSpeeds normalizedSpeeds = kinematics.toChassisSpeeds(moduleStates);

        normalizedSpeeds = ChassisSpeeds::discretize(normalizedSpeeds, deltaTime);
        moduleStates = kinematics.toModuleStates(normalizedSpeeds);

        for (size_t i = 0; i < motors.size(); ++i)
        {
            motors[i].spin(vex::forward, moduleStates[i].velocity / wheelCircumference * 60.0, vex::rpm);
        }
    }

    Pose2d getPose() const
    {
        return poseEstimator.getEstimatedPosition();
    }

    void resetPose(const Pose2d &newPose)
    {
        poseEstimator.resetPose(newPose);
    }

    void setRotationOrigin(const Translation2d &origin)
    {
        kinematics.setRotationOrigin(origin);
    }

    Translation2d getRotationOrigin() const
    {
        return kinematics.getRotationOrigin();
    }

    ChassisSpeeds getChassisSpeeds()
    {
        std::array<XDriveModuleState, _Nm> moduleStates;

        for (size_t i = 0; i < modules.size(); ++i)
        {
            moduleStates[i].velocity = motors[i].velocity(vex::rpm) / 60.0 * wheelCircumference;
            moduleStates[i].angle = modules[i].rotation.value;
        }

        return kinematics.toChassisSpeeds(moduleStates);
    }

    ChassisSpeeds getChassisAcceleration()
    {
        std::array<XDriveModuleState, _Nm> moduleStates;

        for (size_t i = 0; i < modules.size(); ++i)
        {
            moduleStates[i].velocity = accelerations[i] / 60.0 * wheelCircumference;
            moduleStates[i].angle = modules[i].rotation.value;
        }

        return kinematics.toChassisSpeeds(moduleStates);
    }

    std::array<XDriveModulePosition, _Nm> getModulePositions()
    {
        std::array<XDriveModulePosition, _Nm> modulePositions;

        for (size_t i = 0; i < modules.size(); ++i)
        {
            modulePositions[i]
                .distance = motors[i].position(vex::rotationUnits::rev) * wheelCircumference;
            modulePositions[i].angle = modules[i].rotation.value;
        }

        return modulePositions;
    }

    void update(double periodic)
    {
        std::array<XDriveModulePosition, _Nm> modulePositions;

        for (size_t i = 0; i < modules.size(); ++i)
        {
            if (!hasLastVelocities)
            {
                lastVelocities[i] = motors[i].velocity(vex::rpm);
                accelerations[i] = 0.0;
            }
            else
            {
                double currentVelocity = motors[i].velocity(vex::rpm);
                accelerations[i] = (currentVelocity - lastVelocities[i]) / periodic;
                lastVelocities[i] = currentVelocity;
            }

            modulePositions[i]
                .distance = motors[i].position(vex::rotationUnits::rev) * wheelCircumference;
            modulePositions[i].angle = modules[i].rotation.value;
        }

        hasLastVelocities = true;

        poseEstimator.update(getGyroRotation(), modulePositions);
    }

    void addVisionMeasurement(const Pose2d &visionPose, double timestampSeconds, const std::array<double, 3> &visionMeasurementStdDevs)
    {
        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, visionMeasurementStdDevs);
    }

    std::array<vex::motor, _Nm> motors;

private:
    Rotation2d getGyroRotation() const
    {
        if (gyro.installed())
        {
            return Rotation2d{gyro.yaw(vex::rotationUnits::deg) * M_PI / 180.0};
        }
        else
        {
            return Rotation2d{0.0};
        }
    }

    std::array<XDriveModule, _Nm> modules;
    vex::inertial &gyro;

    bool hasLastVelocities = false;
    std::array<double, _Nm> lastVelocities;
    std::array<double, _Nm> accelerations;

    double wheelDiameter;
    double wheelCircumference;
    double attainableMaxSpeedMetersPerSecond;

    XDriveKinematics<_Nm> kinematics;
    XDrivePoseEstimator<_Nm> poseEstimator;
};

#endif