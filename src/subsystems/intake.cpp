#include "subsystems/intake.h"

Intake::Intake()
{
    intakeMotor.setBrake(vex::brake);
    shooterMotor.setBrake(vex::brake);

    CommandScheduler::getInstance()->registerSubsystem(this);
}

void Intake::set(IntakeState state)
{
    if (state == IntakeState::Intake)
    {
        intakeMotor.spin(vex::forward, -200, vex::rpm);
    }
    else if (state == IntakeState::Reverse)
    {
        intakeMotor.spin(vex::forward, 200, vex::rpm);
    }
    else
    {
        intakeMotor.spin(vex::forward, 0, vex::rpm);
    }
}

void Intake::setShooter(IntakeState state)
{
    if (state == IntakeState::Intake)
    {
        shooterMotor.spin(vex::forward, 200, vex::rpm);
    }
    else if (state == IntakeState::Reverse)
    {
        shooterMotor.spin(vex::forward, -200, vex::rpm);
    }
    else
    {
        shooterMotor.spin(vex::forward, 0, vex::rpm);
    }
}

void Intake::periodic()
{
}
