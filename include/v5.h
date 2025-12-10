#ifdef VEX
#include_next "v5.h"
#else

#ifndef V5_H
#define V5_H

#include <functional>

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

    private:
        unsigned long startTime = 0;
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
        red = 0
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
        } ButtonUp, ButtonDown, ButtonLeft, ButtonRight, ButtonA, ButtonB, ButtonX, ButtonY;

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
        PORT8 = 8
    };
};

void postTelemetry(const std::string &path, double value);

#endif

#endif