#include "manualdrive.h"
#include "alliance.h"

static PIDController rPid{4.0, 0.0, 0.2, 0.01};

ManualDrive::ManualDrive(vex::controller *gamepad, Drivetrain *drivetrain)
    : gamepad(gamepad), drivetrain(drivetrain)
{
    addRequirement(drivetrain);
    rPid.enableContinuousInput(-M_PI, M_PI);
}

void ManualDrive::initialize()
{
    rPid.setSetpoint(drivetrain->getPose().rotation.value);
}

void ManualDrive::execute()
{
    ChassisSpeeds speeds;

    double jx = gamepad->Axis3.position(vex::percentUnits::pct) / 100.0;
    double jy = -gamepad->Axis4.position(vex::percentUnits::pct) / 100.0;

    if (vex::getCurrentAlliance() == Alliance::Blue)
    {
        jx = -jx;
        jy = -jy;
    }

    if (rotationOriginMode)
    {
        Translation2d origin = drivetrain->getRotationOrigin();

        origin = origin + Translation2d{
                              jx * 0.01, // x
                              jy * 0.01  // y
                          }
                              .rotateBy(drivetrain->getPose().rotation);

        drivetrain->setRotationOrigin(origin);

        speeds = ChassisSpeeds{
            0,                                                                                       // vx
            0,                                                                                       // vy
            -gamepad->Axis1.position(vex::percentUnits::pct) / 100.0 * Drivetrain::MAX_ANGULAR_SPEED // omega
        };
    }
    else
    {
        speeds = ChassisSpeeds{
            jx * Drivetrain::MAX_SPEED,                                                              // vx
            jy * Drivetrain::MAX_SPEED,                                                              // vy
            -gamepad->Axis1.position(vex::percentUnits::pct) / 100.0 * Drivetrain::MAX_ANGULAR_SPEED // omega
        };
    }

    rPid.setSetpoint(rPid.getSetpoint() + speeds.omega * 0.01);
    speeds.omega = rPid.calculate(drivetrain->getPose().rotation.value);

    drivetrain->drive(ChassisSpeeds::fromFieldRelativeSpeeds(
        speeds,
        drivetrain->getPose().rotation));
}

bool ManualDrive::isFinished()
{
    return false;
}
