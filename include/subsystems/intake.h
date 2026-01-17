#ifndef _INTAKE_H_
#define _INTAKE_H_

#include "vex.h"
#include "commandscheduler.h"

enum class IntakeState
{
    Off,
    Intake,
    Reverse
};

class Intake : public Subsystem
{
public:
    Intake();

    void set(IntakeState state);
    void setShooter(IntakeState state);
    void periodic();

private:
    vex::motor intakeMotor = vex::motor(vex::PORT17, vex::gearSetting::ratio18_1);
    vex::motor shooterMotor = vex::motor(vex::PORT20, vex::gearSetting::ratio18_1);
};

#endif