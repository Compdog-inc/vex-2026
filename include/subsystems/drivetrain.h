#ifndef _DRIVETRAIN_H_
#define _DRIVETRAIN_H_

#include "vex.h"
#include "xdrive.h"
#include "commandscheduler.h"

class Drivetrain : public Subsystem
{
public:
    Drivetrain(vex::inertial &gyro, vex::gps *gps, const Pose2d &initialPose);

    void drive(const ChassisSpeeds &robotRelativeSpeeds);
    void periodic();

    Pose2d getPose();

    void resetPose(const Pose2d &newPose);

    void setRotationOrigin(const Translation2d &origin);

    Translation2d getRotationOrigin();

    ChassisSpeeds getChassisSpeeds();
    ChassisSpeeds getChassisAcceleration();

    static constexpr double TRACK_WIDTH = 0.3429;  // meters
    static constexpr double TRACK_LENGTH = 0.3429; // meters

    static constexpr double MAX_MODULE_SPEED = 1.064;               // meters per second
    static constexpr double MAX_SPEED = MAX_MODULE_SPEED * M_SQRT2; // meters per second
    static constexpr double MAX_ANGULAR_SPEED = 4.388;              // radians per second

private:
    ChassisSpeeds targetSpeeds;
    XDrive<4> xdrive;
    vex::gps *gps;
};

#endif