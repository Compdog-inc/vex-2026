#include "manualdrive.h"

ManualDrive::ManualDrive(vex::controller *gamepad, Drivetrain *drivetrain)
    : gamepad(gamepad), drivetrain(drivetrain)
{
    addRequirement(drivetrain);
}

void ManualDrive::initialize()
{
}

void ManualDrive::execute()
{
    ChassisSpeeds speeds;

    if (rotationOriginMode)
    {
        Translation2d origin = drivetrain->getXDrive()->getRotationOrigin();

        origin = origin + Translation2d{
                              gamepad->Axis4.position(vex::percentUnits::pct) / 100.0 * 0.01, // x
                              gamepad->Axis3.position(vex::percentUnits::pct) / 100.0 * 0.01  // y
                          }
                              .rotateBy(drivetrain->getXDrive()->getPose().rotation);

        drivetrain->getXDrive()->setRotationOrigin(origin);

        speeds = ChassisSpeeds{
            0,                                                                                       // vx
            0,                                                                                       // vy
            -gamepad->Axis1.position(vex::percentUnits::pct) / 100.0 * Drivetrain::MAX_ANGULAR_SPEED // omega
        };
    }
    else
    {
        speeds = ChassisSpeeds{
            gamepad->Axis4.position(vex::percentUnits::pct) / 100.0 * Drivetrain::MAX_SPEED,         // vx
            gamepad->Axis3.position(vex::percentUnits::pct) / 100.0 * Drivetrain::MAX_SPEED,         // vy
            -gamepad->Axis1.position(vex::percentUnits::pct) / 100.0 * Drivetrain::MAX_ANGULAR_SPEED // omega
        };
    }

    drivetrain->drive(ChassisSpeeds::fromFieldRelativeSpeeds(
        speeds,
        drivetrain->getXDrive()->getPose().rotation));
}

bool ManualDrive::isFinished()
{
    return false;
}
