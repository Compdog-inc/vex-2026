#ifndef MANUALINTAKE_H
#define MANUALINTAKE_H

#include "vex.h"
#include "rmath.h"
#include "commandscheduler.h"
#include "subsystems/intake.h"

class ManualIntake : public Command
{
public:
    ManualIntake(vex::controller *gamepad, Intake *intake);

    void initialize() override;
    void execute() override;

    bool isFinished() override;

    bool rotationOriginMode = false;

private:
    vex::controller *gamepad;
    Intake *intake;
};

#endif