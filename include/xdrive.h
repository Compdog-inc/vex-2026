#ifndef __X_DRIVE_H__
#define __X_DRIVE_H__

#include "vex.h"
#include <array>

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
    XDriveOdometry(XDriveKinematics<_Nm> &kinematics)
        : kinematics(kinematics), robotPose(Pose2d{Translation2d{0.0, 0.0}, Rotation2d{0.0}})
    {
    }

    void update(const std::array<XDriveModulePosition, _Nm> &modulePositions)
    {
        Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
        lastModulePositions = modulePositions;

        robotPose = robotPose.exp(twist);
    }

    Pose2d getRobotPose() const
    {
        return robotPose;
    }

    void resetPose(const Pose2d &newPose)
    {
        robotPose = newPose;
    }

private:
    XDriveKinematics<_Nm> &kinematics;
    std::array<XDriveModulePosition, _Nm> lastModulePositions;
    Pose2d robotPose;
};

template <std::size_t _Nm>
class XDrive
{
public:
    XDrive(double wheelDiameter, double attainableMaxSpeedMetersPerSecond, std::array<vex::motor, _Nm> motors, std::array<XDriveModule, _Nm> modules)
        : motors(motors), modules(modules), lastVelocities(), accelerations(), wheelDiameter(wheelDiameter), wheelCircumference(wheelDiameter * M_PI), attainableMaxSpeedMetersPerSecond(attainableMaxSpeedMetersPerSecond), kinematics(modules), odometry(kinematics)
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
        return odometry.getRobotPose();
    }

    void resetPose(const Pose2d &newPose)
    {
        odometry.resetPose(newPose);
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

        odometry.update(modulePositions);
    }

    std::array<vex::motor, _Nm> motors;

private:
    std::array<XDriveModule, _Nm> modules;

    bool hasLastVelocities = false;
    std::array<double, _Nm> lastVelocities;
    std::array<double, _Nm> accelerations;

    double wheelDiameter;
    double wheelCircumference;
    double attainableMaxSpeedMetersPerSecond;

    XDriveKinematics<_Nm> kinematics;
    XDriveOdometry<_Nm> odometry;
};

#endif