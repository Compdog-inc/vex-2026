#include "vex.h"
#include "ruckigalign.h"
#include "alliance.h"
#include "commandscheduler.h"
#include "subsystems/drivetrain.h"
#include "subsystems/intake.h"
#include "manualdrive.h"
#include "manualintake.h"
#include "autointake.h"
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

const Pose2d RED_LEFT_START = Pose2d{Translation2d{-1.6096, 0.5}, Rotation2d{M_PI_2}};
const Pose2d RED_RIGHT_START = Pose2d{Translation2d{-1.6096, -0.5}, Rotation2d{M_PI_2}};
const Pose2d BLUE_LEFT_START = Pose2d{Translation2d{1.6096, -0.5}, Rotation2d{M_PI + M_PI_2}};
const Pose2d BLUE_RIGHT_START = Pose2d{Translation2d{1.6096, 0.5}, Rotation2d{M_PI + M_PI_2}};

vex::inertial gyro = vex::inertial(vex::PORT3);
vex::gps gps = vex::gps(vex::PORT15, -15.0, 153, vex::distanceUnits::mm, 0.0);

static bool isLeft = false;

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
    Pose2d{Translation2d{-0.4, 0.6}, Rotation2d{M_PI_2}},
    Pose2d{Translation2d{-0.4, -0.6}, Rotation2d{M_PI_2}},
    Pose2d{Translation2d{0.4, 0.6}, Rotation2d{M_PI + M_PI_2}},
    Pose2d{Translation2d{0.4, -0.6}, Rotation2d{M_PI + M_PI_2}}};

void autonomous(void);
void usercontrol(void);

static ManualIntake *manualIntake = nullptr;

void robotInit(void)
{
  createSubsystems();

  brain.Screen.setPenColor(vex::color::white);
  brain.Screen.print("Select Starting Position:");
  brain.Screen.newLine();
  brain.Screen.print("A: Red Left");
  brain.Screen.newLine();
  brain.Screen.print("B: Red Right");
  brain.Screen.newLine();
  brain.Screen.print("X: Blue Left");
  brain.Screen.newLine();
  brain.Screen.print("Y: Blue Right");
  brain.Screen.newLine();

  while (true)
  {

    while (
        !gamepad.ButtonA.pressing() &&
        !gamepad.ButtonB.pressing() &&
        !gamepad.ButtonX.pressing() &&
        !gamepad.ButtonY.pressing())
    {
      vex::wait(10, vex::msec);
    }

    brain.Screen.setPenColor(vex::color::yellow);

    if (gamepad.ButtonA.pressing())
    {
      drivetrain->resetPose(RED_LEFT_START);
      brain.Screen.print("Red Left Selected");
      vex::setCurrentAlliance(Alliance::Red);
      isLeft = true;
      break;
    }
    else if (gamepad.ButtonB.pressing())
    {
      drivetrain->resetPose(RED_RIGHT_START);
      brain.Screen.print("Red Right Selected");
      vex::setCurrentAlliance(Alliance::Red);
      isLeft = false;
      break;
    }
    else if (gamepad.ButtonX.pressing())
    {
      drivetrain->resetPose(BLUE_LEFT_START);
      brain.Screen.print("Blue Left Selected");
      vex::setCurrentAlliance(Alliance::Blue);
      isLeft = true;
      break;
    }
    else if (gamepad.ButtonY.pressing())
    {
      drivetrain->resetPose(BLUE_RIGHT_START);
      brain.Screen.print("Blue Right Selected");
      vex::setCurrentAlliance(Alliance::Blue);
      isLeft = false;
      break;
    }
    else
    {
      brain.Screen.print("Unknown Button");
    }
  }

  brain.Screen.newLine();

  RuckigAlign::setup();

  // Commands
  ManualDrive *manualDrive = new ManualDrive(&gamepad, drivetrain);
  manualIntake = new ManualIntake(&gamepad, intake);
  manualDrive->setOnHeap(true);
  manualIntake->setOnHeap(true);

  drivetrain->setDefaultCommand(manualDrive);
  intake->setDefaultCommand(manualIntake);

  gamepad.ButtonY.pressed([]()
                          {
    drivetrain->resetPose(Pose2d{drivetrain->getPose().translation, Rotation2d{0.0}});
    drivetrain->setRotationOrigin(Translation2d{0.0, 0.0}); });

  gamepad.ButtonB.pressed([]()
                          {
    Command *alignCommand = WaypointAlign::alignWithCommand(
        {RED_LEFT_START,
         RED_LEFT_START},
        {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0},
        -1, 0, Commands::none(), [&](const ChassisSpeeds &speeds)
        { drivetrain->drive(speeds); }, []()
        { return RuckigAlign::toKinematicState(drivetrain->getPose(), drivetrain->getChassisSpeeds(), drivetrain->getChassisAcceleration()); }, drivetrain);
    alignCommand->addRequirement(drivetrain);
    alignCommand->setOnHeap(true);
    CommandScheduler::getInstance()->schedule(alignCommand); });

  gamepad.ButtonX.pressed([]()
                          { autonomous(); });

  gamepad.ButtonA.pressed([]()
                          { usercontrol(); });
}

void autonomous(void)
{
  AutoIntake *autoIntake = new AutoIntake(intake);
  autoIntake->setOnHeap(true);

  if (isLeft)
  {
    Command *alignCommand = WaypointAlign::alignWithCommand(
        {vex::getCurrentAlliance() == Alliance::Red ? waypoints[0] : waypoints[3],
         vex::getCurrentAlliance() == Alliance::Red ? waypoints[0] : waypoints[3]},
        {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
        -1, 0, Commands::none(), [&](const ChassisSpeeds &speeds)
        { drivetrain->drive(speeds); }, []()
        { return RuckigAlign::toKinematicState(drivetrain->getPose(), drivetrain->getChassisSpeeds(), drivetrain->getChassisAcceleration()); }, drivetrain);
    alignCommand->addRequirement(drivetrain);
    alignCommand->setOnHeap(true);
    CommandScheduler::getInstance()->schedule(alignCommand->withDeadline({autoIntake->withTimeout(2)}));
  }
  else
  {
    Command *alignCommand = WaypointAlign::alignWithCommand(
        {vex::getCurrentAlliance() == Alliance::Red ? waypoints[1] : waypoints[2],
         vex::getCurrentAlliance() == Alliance::Red ? waypoints[1] : waypoints[2]},
        {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
        -1, 0, Commands::none(), [&](const ChassisSpeeds &speeds)
        { drivetrain->drive(speeds); }, []()
        { return RuckigAlign::toKinematicState(drivetrain->getPose(), drivetrain->getChassisSpeeds(), drivetrain->getChassisAcceleration()); }, drivetrain);
    alignCommand->addRequirement(drivetrain);
    alignCommand->setOnHeap(true);
    CommandScheduler::getInstance()->schedule(alignCommand->withDeadline({autoIntake->withTimeout(2)}));
  }
}

void usercontrol(void)
{
  if (manualIntake != nullptr)
  {
    // CommandScheduler::getInstance()->cancelAll();
    CommandScheduler::getInstance()->schedule(manualIntake);
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

  // Prevent main from exiting with an infinite loop.
  while (true)
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
}
