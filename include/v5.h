#ifdef VEX
#include_next "v5.h"
#else

#ifndef V5_H
#define V5_H

#include <functional>
#include <chrono>

#define M_PI 3.14159265358979323846
#define M_TWOPI 6.28318530717958647692

namespace vex
{
    typedef struct SimulationRegistration
    {
        int id;
        std::function<void()> tickFunction;
    } SimulationRegistration;

    typedef enum timeUnits
    {
        msec = 0,
        sec = 1,
        min = 2,
        hour = 3
    } timeUnits;

    typedef struct timer
    {
        void reset();
        double time(timeUnits units);

        static double systemHighResolution();

    private:
        std::chrono::steady_clock::time_point startTime{std::chrono::steady_clock::now()};
    } timer;

    enum direction
    {
        forward = 0,
        reverse = 1
    };

    enum velocityUnits
    {
        rpm = 0
    };

    enum controllerType
    {
        primary = 0
    };

    enum rotationUnits
    {
        deg = 0,
        rev = 1
    };

    enum distanceUnits
    {
        mm = 0,
        cm = 1,
        in = 3
    };

    enum percentUnits
    {
        pct = 0
    };

    enum gearSetting
    {
        ratio18_1 = 0
    };

    enum brakeType
    {
        coast = 0,
        brake = 1,
        hold = 2
    };

    enum color
    {
        red = 0,
        yellow = 1,
        white = 2
    };

    typedef struct motor
    {
        motor(int port, enum gearSetting gearSetting);
        motor(const motor &m);
        motor &operator=(const motor &m);
        motor(motor &&m);
        motor &operator=(motor &&m);
        ~motor();
        void setBrake(brakeType type);
        void spin(direction direction, double velocity, velocityUnits units);
        double velocity(velocityUnits units);
        double position(rotationUnits units);

    private:
        int port;
        gearSetting gearSetting;
        double currentPosition = 0;
        double currentVelocity = 0;
        int registration = 0;
    } motor;

    void wait(int time, timeUnits units);

    typedef struct brain
    {
        struct Screen
        {
            void setPenColor(color c);
            void print(const char *format, ...);
            void newLine();
        } Screen;
    } brain;

    struct controller
    {
        controller(controllerType type);

        struct Button
        {
            bool isPressed = false;
            bool lastPressed = false;
            void (*onPressed)() = nullptr;
            void pressed(void (*function)());
            bool pressing();
        } ButtonL1,
            ButtonL2,
            ButtonR1,
            ButtonR2,
            ButtonUp,
            ButtonDown,
            ButtonLeft,
            ButtonRight,
            ButtonX,
            ButtonB,
            ButtonY,
            ButtonA;

        struct Axis
        {
            double valuePct = 0.0; // -100..100
            double position(percentUnits units);
        } Axis4, Axis3, Axis2, Axis1;
    };

    typedef struct competition
    {
        void autonomous(void (*function)());
        void drivercontrol(void (*function)());
    } competition;

    enum ports
    {
        PORT1 = 1,
        PORT2 = 2,
        PORT3 = 3,
        PORT4 = 4,
        PORT5 = 5,
        PORT6 = 6,
        PORT7 = 7,
        PORT8 = 8,
        PORT9 = 9,
        PORT10 = 10,
        PORT11 = 11,
        PORT12 = 12,
        PORT13 = 13,
        PORT14 = 14,
        PORT15 = 15,
        PORT16 = 16,
        PORT17 = 17,
        PORT18 = 18,
        PORT19 = 19,
        PORT20 = 20,
        PORT21 = 21,
        PORT22 = 22
    };

    typedef struct inertial
    {
        inertial(int port);
        void calibrate();
        bool isCalibrating();
        bool installed();
        double yaw(rotationUnits units);
        int timestamp();

    private:
        int port;
        bool calibrating = false;
        double yawVal = 0;
        int timestampVal = 0;

        std::chrono::steady_clock::time_point calibrationStartTime{std::chrono::steady_clock::now()};
    } inertial;

    typedef struct gps
    {
        gps(int port, double ox, double oy, distanceUnits distUnits, double oheading);
        void setLocation(double x, double y, distanceUnits distUnits, double heading, rotationUnits rotUnits);
        double xPosition(distanceUnits units);
        double yPosition(distanceUnits units);
        double heading(rotationUnits units);
        void setHeading(double heading, rotationUnits units);
        int quality();
        int timestamp();

        void calibrate();
        bool installed();
        bool isCalibrating();

    private:
        int port;
        double rx = 0;
        double ry = 0;
        double x = 0;
        double y = 0;
        double headingVal = 0;
        int qualityVal = 0;
        int timestampVal = 0;
        bool calibrating = false;

        std::chrono::steady_clock::time_point calibrationStartTime{std::chrono::steady_clock::now()};
        std::chrono::steady_clock::time_point lastUpdateTime{std::chrono::steady_clock::now()};
    } gps;
};

void postTelemetry(const std::string &path, double value);
void postDrivetrainVelocity(double vx, double vy, double omega);
int getSimulationAlliance();

#endif

#endif