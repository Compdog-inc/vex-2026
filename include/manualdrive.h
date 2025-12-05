#ifndef MANUALDRIVE_H
#define MANUALDRIVE_H

#include "vex.h"
#include "rmath.h"
#include "commandscheduler.h"
#include "subsystems/drivetrain.h"

class ManualDrive : public Command
{
public:
    ManualDrive(vex::controller *gamepad, Drivetrain *drivetrain);

    void initialize() override;
    void execute() override;

    bool isFinished() override;

    bool rotationOriginMode = false;

private:
    vex::controller *gamepad;
    Drivetrain *drivetrain;
};

#endif