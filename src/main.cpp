#include "vex.h"
#include "ruckigalign.h"
#include "commandscheduler.h"
#include "subsystems/drivetrain.h"
#include "manualdrive.h"
#include "waypointalign.h"

vex::competition competition;
vex::brain brain = vex::brain();
vex::controller gamepad = vex::controller(vex::controllerType::primary);

void brainPrintWrappedArgs(const char *format, va_list args)
{
  const int maxLineLength = 50;
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);

  char *lineStart = buffer;
  while (*lineStart)
  {
    char *ptr = lineStart;
    int count = 0;
    char *lastSpace = nullptr;

    // Advance until we hit maxLineLength or end; remember last space
    while (*ptr && count < maxLineLength)
    {
      if (*ptr == ' ')
      {
        lastSpace = ptr; // record potential break point
      }
      ptr++;
      count++;
    }

    char *lineEnd = ptr;

    // If we overflowed the width and have a space before overflow, break there
    if (*ptr && count >= maxLineLength && lastSpace && lastSpace >= lineStart)
    {
      lineEnd = lastSpace; // break at last space before overflow
    }
    // Else if we didn't overflow but reached end of string, print to end
    // Else if we overflowed with no space, we'll break hard at max width

    // Print the line
    char saved = *lineEnd;
    *lineEnd = '\0';
    brain.Screen.print("%s", lineStart);
    brain.Screen.newLine();
    *lineEnd = saved;

    // Move start: skip space if we broke at one; otherwise continue
    if (lineEnd == lastSpace)
    {
      lineStart = lineEnd + 1; // skip the space
    }
    else
    {
      lineStart = lineEnd; // continue from where we stopped
    }
  }
}

void brainPrintWrapped(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  brainPrintWrappedArgs(format, args);
  va_end(args);
}

void panic(const char *file, int line, const char *messageFormat, ...)
{
  brain.Screen.setPenColor(vex::color::red);

  va_list args;
  va_start(args, messageFormat);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), messageFormat, args);
  va_end(args);

  brainPrintWrapped("PANIC at %s:%d - %s", file, line, buffer);

  while (true)
  {
    vex::wait(100, vex::timeUnits::msec);
  }
}

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
                            Command *alignCommand = WaypointAlign::alignWithCommand(
                              {
                                Pose2d{Translation2d{0, 0}, Rotation2d{0}},
                                 Pose2d{Translation2d{1, 0}, Rotation2d{0}},
                                  Pose2d{Translation2d{1, 1}, Rotation2d{0}},
                                   Pose2d{Translation2d{0, 1}, Rotation2d{0}},
                                    Pose2d{Translation2d{0.5, 0.5}, Rotation2d{0}}},
                                     {10.0, 10.0, 10.0, 10.0, 10.0}, 
                                     -1, 0, Commands::none(), [&](const ChassisSpeeds &speeds)
                                                                                    { 
                                                                                      drivetrain.drive(speeds); 
                                                                                    }, []()
                                                                                    {
                                                                                      return RuckigAlign::toKinematicState(drivetrain.getXDrive()->getPose(), drivetrain.getXDrive()->getChassisSpeeds(), drivetrain.getXDrive()->getChassisAcceleration());
                                                                                      }, &drivetrain);
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
int main(int argc, char **argv)
{
  // Set up callbacks for autonomous and driver control periods.
  competition.autonomous(autonomous);
  competition.drivercontrol(usercontrol);

  robotInit();

#ifdef VEX
  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    vex::wait(100, vex::msec);
  }
#else
  usercontrol();
#endif
}
