#include "manualintake.h"

ManualIntake::ManualIntake(vex::controller *gamepad, Intake *intake)
    : gamepad(gamepad), intake(intake)
{
    addRequirement(intake);
}

void ManualIntake::initialize()
{
}

void ManualIntake::execute()
{
    if (gamepad->ButtonL2.pressing())
    {
        intake->set(IntakeState::Intake);
    }
    else if (gamepad->ButtonL1.pressing())
    {
        intake->set(IntakeState::Reverse);
    }
    else
    {
        intake->set(IntakeState::Off);
    }
}

bool ManualIntake::isFinished()
{
    return false;
}
