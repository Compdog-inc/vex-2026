#include "subsystems/drivetrain.h"

Drivetrain::Drivetrain(vex::inertial &gyro, vex::gps *gps, const Pose2d &initialPose) : xdrive(
                                                                                            0.1016, // Wheel diameter 101.6 mm (4 inches)
                                                                                            MAX_MODULE_SPEED,
                                                                                            {
                                                                                                vex::motor(vex::PORT7, vex::gearSetting::ratio18_1), // Front Left
                                                                                                vex::motor(vex::PORT8, vex::gearSetting::ratio18_1), // Front Right
                                                                                                vex::motor(vex::PORT5, vex::gearSetting::ratio18_1), // Back Right
                                                                                                vex::motor(vex::PORT6, vex::gearSetting::ratio18_1)  // Back Left
                                                                                            },
                                                                                            {
                                                                                                XDriveModule{Translation2d{-TRACK_WIDTH / 2, TRACK_LENGTH / 2}, Rotation2d{M_PI / 4}},     // Front Left
                                                                                                XDriveModule{Translation2d{TRACK_WIDTH / 2, TRACK_LENGTH / 2}, Rotation2d{-M_PI / 4}},     // Front Right
                                                                                                XDriveModule{Translation2d{TRACK_WIDTH / 2, -TRACK_LENGTH / 2}, Rotation2d{5 * M_PI / 4}}, // Back Right
                                                                                                XDriveModule{Translation2d{-TRACK_WIDTH / 2, -TRACK_LENGTH / 2}, Rotation2d{3 * M_PI / 4}} // Back Left
                                                                                            },
                                                                                            gyro,
                                                                                            initialPose),
                                                                                        gps(gps)
{
    for (int i = 0; i < xdrive.motors.size(); ++i)
    {
        xdrive.motors[i].setBrake(vex::brake);
    }

    gps->setLocation(initialPose.translation.x * 1000.0, initialPose.translation.y * 1000.0, vex::distanceUnits::mm, initialPose.rotation.value * 180.0 / M_PI, vex::rotationUnits::deg);

    CommandScheduler::getInstance()->registerSubsystem(this);
}

void Drivetrain::drive(const ChassisSpeeds &robotRelativeSpeeds)
{
    targetSpeeds = robotRelativeSpeeds;
}

void Drivetrain::periodic()
{
    gps->setHeading(xdrive.getPose().rotation.value * 180.0 / M_PI, vex::rotationUnits::deg);
    int quality = gps->quality();
    if (quality > 90)
    {
        // Map quality 100-90 to stdDev 2.5-3.5 meters
        double stdDevMin = 2.5;
        double stdDevMax = 3.5;
        double stdDev = stdDevMax - (quality - 90) * (stdDevMax - stdDevMin) / 10.0;

        xdrive.addVisionMeasurement(
            Pose2d{
                Translation2d{gps->xPosition(vex::distanceUnits::mm) / 1000.0, gps->yPosition(vex::distanceUnits::mm) / 1000.0},
                Rotation2d{gps->heading(vex::rotationUnits::deg) * M_PI / 180.0}},
            static_cast<double>(gps->timestamp()) / 1000.0,
            {stdDev, stdDev, 999999.0}); // High uncertainty in rotation due to GPS limitations
    }

    xdrive.update(0.01);

#ifndef VEX
    postTelemetry("drivetrain/pose/x", xdrive.getPose().translation.x);
    postTelemetry("drivetrain/pose/y", xdrive.getPose().translation.y);
    postTelemetry("drivetrain/pose/rotation", xdrive.getPose().rotation.value);

    ChassisSpeeds speeds = ChassisSpeeds::fromRobotRelativeSpeeds(getChassisSpeeds(), xdrive.getPose().rotation);
    postDrivetrainVelocity(speeds.vx, speeds.vy, speeds.omega);
#endif

    xdrive.drive(targetSpeeds, 0.01);
}

Pose2d Drivetrain::getPose()
{
    return xdrive.getPose();
}

void Drivetrain::resetPose(const Pose2d &newPose)
{
    xdrive.resetPose(newPose);
    gps->setLocation(newPose.translation.x * 1000.0, newPose.translation.y * 1000.0, vex::distanceUnits::mm, newPose.rotation.value * 180.0 / M_PI, vex::rotationUnits::deg);
}

void Drivetrain::setRotationOrigin(const Translation2d &origin)
{
    xdrive.setRotationOrigin(origin);
}

Translation2d Drivetrain::getRotationOrigin()
{
    return xdrive.getRotationOrigin();
}

ChassisSpeeds Drivetrain::getChassisSpeeds()
{
    return xdrive.getChassisSpeeds();
}

ChassisSpeeds Drivetrain::getChassisAcceleration()
{
    return xdrive.getChassisAcceleration();
}