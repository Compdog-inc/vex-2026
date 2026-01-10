#include "vex.h"
#include "ruckigalign.h"
#include "commandscheduler.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "manualdrive.h"
#include "manualintake.h"
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

vex::inertial gyro = vex::inertial(vex::PORT7);
vex::gps gps = vex::gps(vex::PORT20, 0.0, 0.0, vex::distanceUnits::mm, 180.0);

Drivetrain *drivetrain;
Intake *intake;

// Subsystems
void createSubsystems()
{
  vex::wait(10, vex::timeUnits::msec);

  if (gyro.installed())
  {
    gyro.calibrate();
  }

  if (gps.installed())
  {
    gps.calibrate();
  }

  while ((gyro.installed() && gyro.isCalibrating()) ||
         (gps.installed() && gps.isCalibrating()))
  {
    vex::wait(10, vex::timeUnits::msec);
  }

  drivetrain = new Drivetrain(gyro, &gps, Pose2d::kZero);
  intake = new Intake();
}

std::vector<Pose2d> waypoints = {
    Pose2d{Translation2d{0, 1.9812}, Rotation2d{0}},
    Pose2d{Translation2d{0, 4.826}, Rotation2d{0}},
    Pose2d{Translation2d{-3.0988, 4.826 + 0.254 + 0.127}, Rotation2d{0}},
    Pose2d{Translation2d{-3.0988, 6.985}, Rotation2d{0}},
    Pose2d{Translation2d{0, 6.985}, Rotation2d{0}},
    Pose2d{Translation2d{-3.0988, 1.9812}, Rotation2d{0}},
    Pose2d{Translation2d{-3.0988, 4.826}, Rotation2d{0}},
};

void robotInit(void)
{
  createSubsystems();

  RuckigAlign::setup();

  // Commands
  ManualDrive *manualDrive = new ManualDrive(&gamepad, drivetrain);
  ManualIntake *manualIntake = new ManualIntake(&gamepad, intake);
  manualDrive->setOnHeap(true);
  manualIntake->setOnHeap(true);

  drivetrain->setDefaultCommand(manualDrive);
  intake->setDefaultCommand(manualIntake);

  gamepad.ButtonA.pressed([]()
                          {
    drivetrain->resetPose(Pose2d{drivetrain->getPose().translation, Rotation2d{0.0}});
    drivetrain->setRotationOrigin(Translation2d{0.0, 0.0}); });

  gamepad.ButtonX.pressed([]()
                          {
                            Command *alignCommand = WaypointAlign::alignWithCommand(
                              {
                                waypoints[0],
                                waypoints[1],
                                waypoints[2],
                                waypoints[3],
                                waypoints[4],
                                waypoints[1],
                                waypoints[6],
                                waypoints[5],
                                waypoints[0]
                              },
                                     {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}, 
                                     -1, 0, Commands::none(), [&](const ChassisSpeeds &speeds)
                                                                                    { 
                                                                                      drivetrain->drive(speeds); 
                                                                                    }, []()
                                                                                    {
                                                                                      return RuckigAlign::toKinematicState(drivetrain->getPose(), drivetrain->getChassisSpeeds(), drivetrain->getChassisAcceleration());
                                                                                      }, drivetrain);
                            alignCommand->addRequirement(drivetrain);
                            alignCommand->setOnHeap(true);
                            CommandScheduler::getInstance()->schedule(alignCommand); });
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

#ifndef VEX
    postTelemetry("system/loop_time_ms", elapsed);
#endif

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
