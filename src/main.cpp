#include "vex.h"
#include "ruckigalign.h"
#include "commandscheduler.h"
#include "subsystems/drivetrain.h"
#include "manualdrive.h"

vex::competition competition;
vex::brain brain = vex::brain();
vex::controller gamepad = vex::controller(vex::controllerType::primary);

// Subsystems
Drivetrain drivetrain = Drivetrain();

// Commands
ManualDrive manualDrive = ManualDrive(&gamepad, &drivetrain);

double xTarget = 0.0;
double yTarget = 0.0;
double angleTarget = 0.0;

void robotInit(void)
{
  RuckigAlign::setup();

  drivetrain.setDefaultCommand(&manualDrive);

  gamepad.ButtonA.pressed([]()
                          {
    drivetrain.getXDrive()->resetPose(Pose2d{drivetrain.getXDrive()->getPose().translation, Rotation2d{0.0}});
    drivetrain.getXDrive()->setRotationOrigin(Translation2d{0.0, 0.0}); });

  gamepad.ButtonB.pressed([]()
                          { manualDrive.rotationOriginMode = !manualDrive.rotationOriginMode; });

  gamepad.ButtonX.pressed([]()
                          { 
                RuckigAlign* alignCommand = new RuckigAlign(
                    [&]() {
                        return RuckigAlign::toKinematicState(
                            drivetrain.getXDrive()->getPose(),
                            drivetrain.getXDrive()->getChassisSpeeds(),
                            drivetrain.getXDrive()->getChassisAcceleration());
                    },
                    [&]() {
                        return RuckigAlignState{
                            KinematicState{
                                {xTarget, yTarget, angleTarget},
                                {0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0}},
                            AlignMode::Position
                        };
                    },
                    [&](const ChassisSpeeds &speeds) {
                        drivetrain.drive(speeds);
                    },
                    true);
                alignCommand->addRequirement(&drivetrain);
                alignCommand->setOnHeap(true);
                CommandScheduler::getInstance()->schedule(alignCommand); });

  gamepad.ButtonUp.pressed([]()
                           { 
                            xTarget = -1.0;
                            yTarget = 1.0;
                            angleTarget = -M_PI / 2; });
  gamepad.ButtonDown.pressed([]()
                             { 
                            xTarget = 1.0;
                            yTarget = -1.0;
                            angleTarget = M_PI / 2; });

  gamepad.ButtonRight.pressed([]()
                              { 
                              xTarget = -1.0;
                              yTarget = -1.0;
                              angleTarget = -M_PI; });

  gamepad.ButtonLeft.pressed([]()
                             { 
                              xTarget = 1.0;
                              yTarget = 1.0;
                              angleTarget = M_PI; });
}

void autonomous(void)
{
}

void usercontrol(void)
{
  vex::timer periodicTimer = vex::timer();

  while (1)
  {
    periodicTimer.reset();

    CommandScheduler::getInstance()->run();

    double elapsed = periodicTimer.time(vex::timeUnits::msec);
    if (elapsed < 10)
    {
      vex::wait(10 - elapsed, vex::timeUnits::msec);
    }
    else
    {
      brain.Screen.setPenColor(vex::color::red);
      brain.Screen.print("Loop overrun: %.2f ms", elapsed);
      brain.Screen.newLine();
      vex::wait(1, vex::timeUnits::msec);
    }
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  competition.autonomous(autonomous);
  competition.drivercontrol(usercontrol);

  robotInit();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    vex::wait(100, vex::msec);
  }
}
