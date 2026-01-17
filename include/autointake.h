#ifndef AUTOINTAKE_H
#define AUTOINTAKE_H

#include "vex.h"
#include "rmath.h"
#include "commandscheduler.h"
#include "subsystems/intake.h"

class AutoIntake : public Command
{
public:
    AutoIntake(Intake *intake);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() override;

private:
    Intake *intake;
};

#endif