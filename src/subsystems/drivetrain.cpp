#include "subsystems/drivetrain.h"

Drivetrain::Drivetrain() : xdrive(
                               0.1016, // Wheel diameter 101.6 mm (4 inches)
                               MAX_MODULE_SPEED,
                               {
                                   vex::motor(vex::PORT5, vex::gearSetting::ratio18_1), // Front Left
                                   vex::motor(vex::PORT6, vex::gearSetting::ratio18_1), // Front Right
                                   vex::motor(vex::PORT7, vex::gearSetting::ratio18_1), // Back Right
                                   vex::motor(vex::PORT8, vex::gearSetting::ratio18_1)  // Back Left
                               },
                               {
                                   XDriveModule{Translation2d{-TRACK_WIDTH / 2, TRACK_LENGTH / 2}, Rotation2d{M_PI / 4}},     // Front Left
                                   XDriveModule{Translation2d{TRACK_WIDTH / 2, TRACK_LENGTH / 2}, Rotation2d{-M_PI / 4}},     // Front Right
                                   XDriveModule{Translation2d{TRACK_WIDTH / 2, -TRACK_LENGTH / 2}, Rotation2d{5 * M_PI / 4}}, // Back Right
                                   XDriveModule{Translation2d{-TRACK_WIDTH / 2, -TRACK_LENGTH / 2}, Rotation2d{3 * M_PI / 4}} // Back Left
                               })
{
    for (int i = 0; i < xdrive.motors.size(); ++i)
    {
        xdrive.motors[i].setBrake(vex::brake);
    }

    CommandScheduler::getInstance()->registerSubsystem(this);
}

void Drivetrain::drive(const ChassisSpeeds &robotRelativeSpeeds)
{
    targetSpeeds = robotRelativeSpeeds;
}

void Drivetrain::periodic()
{
    xdrive.update(0.01);

#ifndef VEX
    postTelemetry("drivetrain/pose/x", xdrive.getPose().translation.x);
    postTelemetry("drivetrain/pose/y", xdrive.getPose().translation.y);
    postTelemetry("drivetrain/pose/rotation", xdrive.getPose().rotation.value);
#endif

    xdrive.drive(targetSpeeds, 0.01);
}

XDrive<4> *Drivetrain::getXDrive()
{
    return &xdrive;
}