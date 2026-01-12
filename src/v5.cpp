#ifndef VEX
#include "v5.h"
#include "panic.h"
#include <chrono>
#include <stdio.h>
#include <vector>
#include <map>
#include <string>
#include <functional>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <shared_mutex>
#include <mutex>
#include <thread>
#include <atomic>
#include <new>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <random>
#if defined(__APPLE__)
#include <sys/resource.h>
#if __has_include(<mach/mach.h>)
#include <mach/mach.h>
#endif
#endif
#include "App.h"

#define SIM_DELTA_TIME 0.01
// Resident Set Size memory cap in bytes (default 100 MB)
#ifndef RSS_MEMORY_CAP
#define RSS_MEMORY_CAP (15ULL * 1024ULL * 1024ULL)
#endif

static double robotXVelocity = 0.0;
static double robotYVelocity = 0.0;
static double robotOmegaVelocity = 0.0;

static std::vector<vex::SimulationRegistration> &getSimulationTicks()
{
    static std::vector<vex::SimulationRegistration> simulationTicks;
    return simulationTicks;
}

static std::map<std::string, double> &getTelemetryMap()
{
    static std::map<std::string, double> telemetryMap;
    return telemetryMap;
}

// Keyboard control state
static std::atomic<bool> g_controlRunning{false};
static std::thread g_controlThread;

static std::mutex &getSimulationTicksMutex()
{
    static std::mutex simulationTicksMutex;
    return simulationTicksMutex;
}

static std::mutex &getTelemetryMapMutex()
{
    static std::mutex telemetryMapMutex;
    return telemetryMapMutex;
}

vex::SimulationRegistration registerSimulation(std::function<void()> tickFunction)
{
    // generate id
    static std::atomic<int> nextId{1};
    int id = nextId.fetch_add(1);
    vex::SimulationRegistration registration = {id, tickFunction};

    std::lock_guard<std::mutex> lock(getSimulationTicksMutex());
    getSimulationTicks().push_back(registration);

    return registration;
}

bool unregisterSimulation(int registrationId)
{
    std::lock_guard<std::mutex> lock(getSimulationTicksMutex());
    auto &ticks = getSimulationTicks();
    auto it = std::remove_if(ticks.begin(), ticks.end(),
                             [registrationId](const vex::SimulationRegistration &reg)
                             { return reg.id == registrationId; });
    bool removed = it != ticks.end();
    ticks.erase(it, ticks.end());
    return removed;
}

void postTelemetry(const std::string &path, double value)
{
    std::lock_guard<std::mutex> lock(getTelemetryMapMutex());
    getTelemetryMap()[path] = value;
}

void postDrivetrainVelocity(double vx, double vy, double omega)
{
    robotXVelocity = vx;
    robotYVelocity = vy;
    robotOmegaVelocity = omega;
}

bool queryTelemetry(const std::string &path, double *outValue)
{
    std::lock_guard<std::mutex> lock(getTelemetryMapMutex());
    auto &map = getTelemetryMap();
    auto it = map.find(path);
    if (it != map.end())
    {
        *outValue = it->second;
        return true;
    }
    return false;
}

double queryTelemetry(const std::string &path)
{
    std::lock_guard<std::mutex> lock(getTelemetryMapMutex());
    auto &map = getTelemetryMap();
    auto it = map.find(path);
    if (it != map.end())
    {
        return it->second;
    }
    return 0.0;
}

double atomicIncrementTelemetry(const std::string &path, double increment)
{
    std::lock_guard<std::mutex> lock(getTelemetryMapMutex());
    auto &map = getTelemetryMap();
    double &value = map[path];
    value += increment;
    return value;
}

std::vector<std::pair<std::string, double>> getAllTelemetryValues()
{
    std::lock_guard<std::mutex> lock(getTelemetryMapMutex());
    std::vector<std::pair<std::string, double>> paths;
    for (const auto &entry : getTelemetryMap())
    {
        paths.push_back({entry.first, entry.second});
    }
    return paths;
}

void vex::timer::reset()
{
    startTime = std::chrono::steady_clock::now();
}

double vex::timer::time(vex::timeUnits units)
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - startTime;

    switch (units)
    {
    case vex::msec:
        return std::chrono::duration<double, std::milli>(elapsed).count();
    case vex::sec:
        return std::chrono::duration<double>(elapsed).count();
    case vex::min:
        return std::chrono::duration<double, std::ratio<60>>(elapsed).count();
    case vex::hour:
        return std::chrono::duration<double, std::ratio<3600>>(elapsed).count();
    default:
        return std::chrono::duration<double, std::milli>(elapsed).count();
    }
}

double vex::timer::systemHighResolution()
{
    static auto systemStartTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - systemStartTime;
    return std::chrono::duration<double, std::milli>(elapsed).count();
}

vex::motor::motor(int port, enum gearSetting gearSetting) : port(port),
                                                            gearSetting(gearSetting)
{
    registration = registerSimulation([this, port]()
                                      { 
                        this->currentPosition += this->currentVelocity * SIM_DELTA_TIME / 60.0;
                        postTelemetry("motor/" + std::to_string(port) + "/position", this->currentPosition);
                        postTelemetry("motor/" + std::to_string(port) + "/velocity", this->currentVelocity); })
                       .id;
    atomicIncrementTelemetry("counters/motor", 1);
    postTelemetry("motor/" + std::to_string(port) + "/position", this->currentPosition);
    postTelemetry("motor/" + std::to_string(port) + "/velocity", this->currentVelocity);
}

vex::motor::motor(const vex::motor &m) : vex::motor(m.port, m.gearSetting)
{
}

vex::motor &vex::motor::operator=(const vex::motor &m)
{
    if (this != &m)
    {
        // Unregister current simulation if it exists
        if (unregisterSimulation(registration))
        {
            atomicIncrementTelemetry("counters/motor", -1);
        }

        // Copy the basic properties
        port = m.port;
        gearSetting = m.gearSetting;
        currentPosition = m.currentPosition;
        currentVelocity = m.currentVelocity;

        // Register new simulation with copied port
        registration = registerSimulation([this, port = this->port]()
                                          {
            this->currentPosition += this->currentVelocity * SIM_DELTA_TIME / 60.0;
            postTelemetry("motor/" + std::to_string(port) + "/position", this->currentPosition);
            postTelemetry("motor/" + std::to_string(port) + "/velocity", this->currentVelocity); })
                           .id;

        // Update telemetry counters and initial values
        atomicIncrementTelemetry("counters/motor", 1);
        postTelemetry("motor/" + std::to_string(this->port) + "/position", this->currentPosition);
        postTelemetry("motor/" + std::to_string(this->port) + "/velocity", this->currentVelocity);
    }
    return *this;
}

vex::motor::motor(vex::motor &&m) : vex::motor(m.port, m.gearSetting)
{
}

vex::motor &vex::motor::operator=(vex::motor &&m)
{
    if (this != &m)
    {
        // Unregister current simulation if it exists
        if (unregisterSimulation(registration))
        {
            atomicIncrementTelemetry("counters/motor", -1);
        }

        // Move the basic properties
        port = m.port;
        gearSetting = m.gearSetting;
        currentPosition = m.currentPosition;
        currentVelocity = m.currentVelocity;

        // Register new simulation with moved port
        registration = registerSimulation([this, port = this->port]()
                                          {
            this->currentPosition += this->currentVelocity * SIM_DELTA_TIME / 60.0;
            postTelemetry("motor/" + std::to_string(port) + "/position", this->currentPosition);
            postTelemetry("motor/" + std::to_string(port) + "/velocity", this->currentVelocity); })
                           .id;

        // Update telemetry counters and initial values
        atomicIncrementTelemetry("counters/motor", 1);
        postTelemetry("motor/" + std::to_string(this->port) + "/position", this->currentPosition);
        postTelemetry("motor/" + std::to_string(this->port) + "/velocity", this->currentVelocity);
    }
    return *this;
}

vex::motor::~motor()
{
    if (unregisterSimulation(registration))
    {
        atomicIncrementTelemetry("counters/motor", -1);
    }
}

void vex::motor::setBrake(brakeType type)
{
}

void vex::motor::spin(direction direction, double velocity, velocityUnits units)
{
    currentVelocity = velocity;
}

double vex::motor::velocity(velocityUnits units)
{
    return currentVelocity;
}

double vex::motor::position(rotationUnits units)
{
    return currentPosition;
}

vex::inertial::inertial(int port) : port(port)
{
    atomicIncrementTelemetry("counters/inertial", 1);
    postTelemetry("inertial/" + std::to_string(port) + "/calibrating", 0.0);
    postTelemetry("inertial/" + std::to_string(port) + "/installed", 1.0);
    postTelemetry("inertial/" + std::to_string(port) + "/yaw", 0.0);
    postTelemetry("inertial/" + std::to_string(port) + "/timestamp", 0.0);

    registerSimulation([this, port]()
                       { 
        // Simulate yaw changes if needed
        yawVal += robotOmegaVelocity * SIM_DELTA_TIME;

        postTelemetry("inertial/" + std::to_string(port) + "/yaw", this->yawVal);

        postTelemetry("inertial/" + std::to_string(port) + "/timestamp", this->timestampVal);

        if(calibrating)
        {
            double elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - calibrationStartTime).count();
            if(elapsedSeconds >= 1.5){ // assume calibration takes 1.5 seconds
                calibrating = false;
                postTelemetry("inertial/" + std::to_string(port) + "/calibrating", 0.0);
            }
        } 
    
        timestampVal = static_cast<int>(vex::timer::systemHighResolution()); });
}

void vex::inertial::calibrate()
{
    calibrating = true;
    calibrationStartTime = std::chrono::steady_clock::now();
    postTelemetry("inertial/" + std::to_string(port) + "/calibrating", 1.0);
}

bool vex::inertial::isCalibrating()
{
    return calibrating;
}

bool vex::inertial::installed()
{
    return true;
}

double vex::inertial::yaw(rotationUnits units)
{
    return yawVal * (units == vex::deg ? 180.0 / M_PI : 1.0 / (2.0 * M_PI));
}

int vex::inertial::timestamp()
{
    return timestampVal;
}

double gaussianRandom(double mean, double stddev)
{
    static thread_local std::mt19937 generator(std::random_device{}());
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator);
}

vex::gps::gps(int port, double ox, double oy, distanceUnits distUnits, double oheading) : port(port)
{
    atomicIncrementTelemetry("counters/gps", 1);
    postTelemetry("gps/" + std::to_string(port) + "/calibrating", 0.0);
    postTelemetry("gps/" + std::to_string(port) + "/installed", 1.0);
    postTelemetry("gps/" + std::to_string(port) + "/x", 0.0);
    postTelemetry("gps/" + std::to_string(port) + "/y", 0.0);
    postTelemetry("gps/" + std::to_string(port) + "/heading", 0.0);
    postTelemetry("gps/" + std::to_string(port) + "/quality", 0.0);
    postTelemetry("gps/" + std::to_string(port) + "/timestamp", 0.0);

    registerSimulation([this, port]()
                       {
                        rx += robotXVelocity * SIM_DELTA_TIME;
                        ry += robotYVelocity * SIM_DELTA_TIME;

                           // Simulate yaw changes if needed
                           postTelemetry("gps/" + std::to_string(port) + "/x", this->x);
                           postTelemetry("gps/" + std::to_string(port) + "/y", this->y);
                           postTelemetry("gps/" + std::to_string(port) + "/heading", this->headingVal);
                           postTelemetry("gps/" + std::to_string(port) + "/quality", this->qualityVal);
                           postTelemetry("gps/" + std::to_string(port) + "/timestamp", this->timestampVal);

                           if (calibrating)
                           {
                               double elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - calibrationStartTime).count();
                               if (elapsedSeconds >= 2.5)
                               { // assume calibration takes 2.5 seconds
                                   calibrating = false;
                                   postTelemetry("gps/" + std::to_string(port) + "/calibrating", 0.0);
                               }
                           }

                           if (installed())
                           {
                               const double UPDATE_INTERVAL = 1.0 / 8.0; // 8 FPS
                               double elapsedSinceLastUpdate = std::chrono::duration<double>(std::chrono::steady_clock::now() - lastUpdateTime).count();
                               if (elapsedSinceLastUpdate >= UPDATE_INTERVAL)
                               {
                                double nx = rx;
                                double ny = ry;

                                      nx += gaussianRandom(0.0, 0.02); // add some noise
                                      ny += gaussianRandom(0.0, 0.02); // add some noise

                                      this->x = nx;
                                        this->y = ny;

                                   lastUpdateTime = std::chrono::steady_clock::now();
                                   // Simulate GPS quality and timestamp updates
                                   qualityVal = 95;                                          // fixed quality for simulation
                                   timestampVal = static_cast<int>(vex::timer::systemHighResolution());
                               }
                           } });
}

void vex::gps::setLocation(double x, double y, distanceUnits distUnits, double heading, rotationUnits rotUnits)
{
    rx = x = x;
    ry = y = y;
    headingVal = heading / (rotUnits == vex::deg ? 180.0 / M_PI : 1.0 / (2.0 * M_PI));
    postTelemetry("gps/" + std::to_string(port) + "/x", x);
    postTelemetry("gps/" + std::to_string(port) + "/y", y);
    postTelemetry("gps/" + std::to_string(port) + "/heading", headingVal);
}

double vex::gps::xPosition(distanceUnits units)
{
    return x * (units == vex::mm ? 1000.0 : units == vex::cm ? 100.0
                                                             : 39.37);
}

double vex::gps::yPosition(distanceUnits units)
{
    return y * (units == vex::mm ? 1000.0 : units == vex::cm ? 100.0
                                                             : 39.37);
}

double vex::gps::heading(rotationUnits units)
{
    return headingVal * (units == vex::deg ? 180.0 / M_PI : 1.0 / (2.0 * M_PI));
}

void vex::gps::setHeading(double heading, rotationUnits units)
{
    headingVal = heading / (units == vex::deg ? 180.0 / M_PI : 1.0 / (2.0 * M_PI));
    postTelemetry("gps/" + std::to_string(port) + "/heading", headingVal);
}

int vex::gps::quality()
{
    return qualityVal;
}

int vex::gps::timestamp()
{
    return timestampVal;
}

bool vex::gps::isCalibrating()
{
    return calibrating;
}

void vex::gps::calibrate()
{
    calibrating = true;
    calibrationStartTime = std::chrono::steady_clock::now();
    postTelemetry("gps/" + std::to_string(port) + "/calibrating", 1.0);
}

bool vex::gps::installed()
{
    return true;
}

void vex::wait(int time, timeUnits units)
{
    unsigned long waitMillis = 0;
    switch (units)
    {
    case vex::msec:
        waitMillis = time;
        break;
    case vex::sec:
        waitMillis = time * 1000;
        break;
    case vex::min:
        waitMillis = time * 1000 * 60;
        break;
    case vex::hour:
        waitMillis = time * 1000 * 3600;
        break;
    default:
        waitMillis = time;
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(waitMillis));
}

void vex::brain::Screen::setPenColor(color c)
{
    // use console escape codes and print them
    switch (c)
    {
    case vex::color::red:
        printf("\033[31m"); // Set text color to red
        break;
    // Add more colors as needed
    default:
        printf("\033[0m"); // Reset to default
        break;
    }
}

void vex::brain::Screen::print(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

void vex::brain::Screen::newLine()
{
    printf("\n");
}

static double &sharedAxisValue(int idx)
{
    static double axes[4] = {0, 0, 0, 0};
    if (idx >= 1 && idx <= 4)
        return axes[idx - 1];
    static double dummy = 0; // fallback
    return dummy;
}

static void setSharedAxisPct(int idx, double pct)
{
    pct = std::max(-100.0, std::min(100.0, pct));
    sharedAxisValue(idx) = pct;
}

static bool &sharedButtonValue(int buttonIndex)
{
    static bool buttons[12] = {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false};

    if (buttonIndex >= 1 && buttonIndex <= 12)
        return buttons[buttonIndex - 1];

    static bool dummy = 0;
    return dummy;
}

static void triggerSharedButton(int buttonIndex)
{
    sharedButtonValue(buttonIndex) = true;
}

static void resetSharedButton(int buttonIndex)
{
    sharedButtonValue(buttonIndex) = false;
}

// Process incoming control packets from WebSocket clients.
// Expected JSON (compact):
// {"type":"axis","index":<1..4>,"value":<-100..100>} OR
// {"type":"button","name":"A|B|X|Y|Up|Down|Left|Right","pressed":true|false}
static void processIncomingPacket(const std::string &json)
{
    // Minimal parsing for known fields without external dependencies.
    if (json.find("\"type\":\"axis\"") != std::string::npos)
    {
        int index = 0;
        double value = 0.0;
        {
            auto p = json.find("\"index\":");
            if (p != std::string::npos)
            {
                std::stringstream ss(json.substr(p + 8));
                ss >> index;
            }
        }
        {
            auto p = json.find("\"value\":");
            if (p != std::string::npos)
            {
                std::stringstream ss(json.substr(p + 8));
                ss >> value;
            }
        }
        if (index >= 1 && index <= 4)
        {
            setSharedAxisPct(index, value);
        }
        return;
    }

    if (json.find("\"type\":\"button\"") != std::string::npos)
    {
        std::string name;
        bool pressed = false;
        {
            auto p = json.find("\"name\":\"");
            if (p != std::string::npos)
            {
                auto start = p + 8;
                auto end = json.find('"', start);
                if (end != std::string::npos)
                    name = json.substr(start, end - start);
            }
        }
        {
            auto p = json.find("\"pressed\":");
            if (p != std::string::npos)
            {
                auto v = json.substr(p + 10);
                pressed = v.find("true") != std::string::npos;
            }
        }

        auto toIndex = [](const std::string &n) -> int
        {
            if (n == "Up")
                return 1;
            if (n == "Down")
                return 2;
            if (n == "Left")
                return 3;
            if (n == "Right")
                return 4;
            if (n == "A")
                return 5;
            if (n == "B")
                return 6;
            if (n == "X")
                return 7;
            if (n == "Y")
                return 8;
            if (n == "L1")
                return 9;
            if (n == "L2")
                return 10;
            if (n == "R1")
                return 11;
            if (n == "R2")
                return 12;
            return 0;
        };
        int idx = toIndex(name);
        if (idx >= 1 && idx <= 12)
        {
            if (pressed)
                triggerSharedButton(idx);
            else
                resetSharedButton(idx);
        }
        return;
    }
}

vex::controller::controller(controllerType type)
{
    registerSimulation([&]()
                       {
        // Update all axes from shared state
        this->Axis1.valuePct = sharedAxisValue(1);
        this->Axis2.valuePct = sharedAxisValue(2);
        this->Axis3.valuePct = sharedAxisValue(3);
        this->Axis4.valuePct = sharedAxisValue(4);

        this->ButtonUp.isPressed = sharedButtonValue(1);
        this->ButtonDown.isPressed = sharedButtonValue(2);
        this->ButtonLeft.isPressed = sharedButtonValue(3);
        this->ButtonRight.isPressed = sharedButtonValue(4);
        this->ButtonA.isPressed = sharedButtonValue(5);
        this->ButtonB.isPressed = sharedButtonValue(6);
        this->ButtonX.isPressed = sharedButtonValue(7);
        this->ButtonY.isPressed = sharedButtonValue(8);
        this->ButtonL1.isPressed = sharedButtonValue(9);
        this->ButtonL2.isPressed = sharedButtonValue(10);
        this->ButtonR1.isPressed = sharedButtonValue(11);
        this->ButtonR2.isPressed = sharedButtonValue(12);

        if (this->ButtonUp.isPressed && !this->ButtonUp.lastPressed)
        {
            if(this->ButtonUp.onPressed)
                this->ButtonUp.onPressed();
            atomicIncrementTelemetry("controller/ButtonUp", 1);
        }
        if (this->ButtonDown.isPressed && !this->ButtonDown.lastPressed)
        {
            atomicIncrementTelemetry("controller/ButtonDown", 1);
            if(this->ButtonDown.onPressed)
                this->ButtonDown.onPressed();
        }
        if (this->ButtonLeft.isPressed && !this->ButtonLeft.lastPressed)
        {
            atomicIncrementTelemetry("controller/ButtonLeft", 1);
            if(this->ButtonLeft.onPressed)
            this->ButtonLeft.onPressed();
        }
        if (this->ButtonRight.isPressed && !this->ButtonRight.lastPressed)
        {
            if(this->ButtonRight.onPressed)
            this->ButtonRight.onPressed();
            atomicIncrementTelemetry("controller/ButtonRight", 1);
        }
        if (this->ButtonA.isPressed && !this->ButtonA.lastPressed)
        {
            if(this->ButtonA.onPressed)
            this->ButtonA.onPressed();
            atomicIncrementTelemetry("controller/ButtonA", 1);
        }
        if (this->ButtonB.isPressed && !this->ButtonB.lastPressed)
        {
            if(this->ButtonB.onPressed)
            this->ButtonB.onPressed();
            atomicIncrementTelemetry("controller/ButtonB", 1);
        }
        if (this->ButtonX.isPressed && !this->ButtonX.lastPressed)
        {
            if(this->ButtonX.onPressed)
            this->ButtonX.onPressed();
            atomicIncrementTelemetry("controller/ButtonX", 1);
        }
        if (this->ButtonY.isPressed && !this->ButtonY.lastPressed)
        {
            if(this->ButtonY.onPressed)
                this->ButtonY.onPressed();
            atomicIncrementTelemetry("controller/ButtonY", 1);
        }
        if (this->ButtonL1.isPressed && !this->ButtonL1.lastPressed)
        {
            if(this->ButtonL1.onPressed)
                this->ButtonL1.onPressed();
            atomicIncrementTelemetry("controller/ButtonL1", 1);
        }
        if (this->ButtonL2.isPressed && !this->ButtonL2.lastPressed)
        {
            if(this->ButtonL2.onPressed)
                this->ButtonL2.onPressed();
            atomicIncrementTelemetry("controller/ButtonL2", 1);
        }
        if (this->ButtonR1.isPressed && !this->ButtonR1.lastPressed)
        {
            if(this->ButtonR1.onPressed)
                this->ButtonR1.onPressed();
            atomicIncrementTelemetry("controller/ButtonR1", 1);
        }
        if (this->ButtonR2.isPressed && !this->ButtonR2.lastPressed)
        {
            if(this->ButtonR2.onPressed)
                this->ButtonR2.onPressed();
            atomicIncrementTelemetry("controller/ButtonR2", 1);
        }

        this->ButtonUp.lastPressed = this->ButtonUp.isPressed;
        this->ButtonDown.lastPressed = this->ButtonDown.isPressed;
        this->ButtonLeft.lastPressed = this->ButtonLeft.isPressed;
        this->ButtonRight.lastPressed = this->ButtonRight.isPressed;
        this->ButtonA.lastPressed = this->ButtonA.isPressed;
        this->ButtonB.lastPressed = this->ButtonB.isPressed;
        this->ButtonX.lastPressed = this->ButtonX.isPressed;
        this->ButtonY.lastPressed = this->ButtonY.isPressed;
        this->ButtonL1.lastPressed = this->ButtonL1.isPressed;
        this->ButtonL2.lastPressed = this->ButtonL2.isPressed;
        this->ButtonR1.lastPressed = this->ButtonR1.isPressed;
        this->ButtonR2.lastPressed = this->ButtonR2.isPressed;

        // Telemtry
        postTelemetry("controller/Axis1", this->Axis1.valuePct);
        postTelemetry("controller/Axis2", this->Axis2.valuePct);
        postTelemetry("controller/Axis3", this->Axis3.valuePct);
        postTelemetry("controller/Axis4", this->Axis4.valuePct); });
}

double vex::controller::Axis::position(percentUnits units)
{
    return valuePct;
}

bool vex::controller::Button::pressing()
{
    return isPressed;
}

void vex::controller::Button::pressed(void (*function)())
{
    onPressed = function;
}

void vex::competition::autonomous(void (*autonomousFunction)())
{
}

void simulationThread()
{
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(getSimulationTicksMutex());
            for (auto &registration : getSimulationTicks())
            {
                registration.tickFunction();
            }

            for (int i = 1; i <= 8; i++)
            {
                resetSharedButton(i);
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(SIM_DELTA_TIME));
    }
}

// Keyboard control thread for --control mode
void keyboardControlThread()
{
    // Raw, non-blocking input
    struct RawNB
    {
        termios orig;
        bool enabled{false};
        int flags{0};
        RawNB()
        {
            if (tcgetattr(STDIN_FILENO, &orig) == 0)
            {
                termios raw = orig;
                raw.c_lflag &= ~(ICANON | ECHO | IEXTEN);
                raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
                raw.c_cflag |= (CS8);
                raw.c_cc[VMIN] = 0;
                raw.c_cc[VTIME] = 1;
                if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == 0)
                {
                    enabled = true;
                }
            }
            int f = fcntl(STDIN_FILENO, F_GETFL, 0);
            if (f == -1)
                f = 0;
            flags = f;
            fcntl(STDIN_FILENO, F_SETFL, f | O_NONBLOCK);
        }
        ~RawNB()
        {
            // Do NOT restore termios here to avoid racing with interactive terminal
            fcntl(STDIN_FILENO, F_SETFL, flags);
        }
    } guard;

    printf("\033[2J\033[H");
    printf("Keyboard Control Active (--control)\n");
    printf("Arrows=Up/Down/Left/Right buttons; A/B/X/Y= 'e'/'b'/'x'/'y'\n");
    printf("Axes: WSAD -> Axis1/Axis2, IJKL -> Axis3/Axis4 (percent). Release returns to 0. Multiple keys supported.\n");
    printf("Press 'q' to exit keyboard control mode.\n\n");

    // Use shared controller state across all controllers.

    // Track last seen timestamps for axis keys
    std::map<char, std::chrono::steady_clock::time_point> lastKey;
    const auto nowTP = []()
    { return std::chrono::steady_clock::now(); };
    const double holdPct = 100.0;
    const std::chrono::milliseconds releaseTimeout(200); // if no repeat for 200ms, consider released

    g_controlRunning.store(true);
    while (g_controlRunning.load())
    {
        unsigned char c = 0;
        ssize_t r = read(STDIN_FILENO, &c, 1);
        if (r > 0)
        {
            if (c == 'q')
            {
                g_controlRunning.store(false);
                break; // exit control
            }
            if (c == 'e')
                triggerSharedButton(5); // A
            else if (c == 'b')
                triggerSharedButton(6);
            else if (c == 'x')
                triggerSharedButton(7);
            else if (c == 'y')
                triggerSharedButton(8);
            else if (c == 'w' || c == 's' || c == 'a' || c == 'd' || c == 'i' || c == 'j' || c == 'k' || c == 'l')
            {
                lastKey[(char)c] = nowTP();
            }
            else if (c == '\x1b')
            {
                unsigned char seq[2] = {0, 0};
                if (read(STDIN_FILENO, &seq[0], 1) <= 0)
                    continue;
                if (read(STDIN_FILENO, &seq[1], 1) <= 0)
                    continue;
                if (seq[0] == '[')
                {
                    if (seq[1] == 'A')
                        triggerSharedButton(1);
                    else if (seq[1] == 'B')
                        triggerSharedButton(2);
                    else if (seq[1] == 'D')
                        triggerSharedButton(3);
                    else if (seq[1] == 'C')
                        triggerSharedButton(4);
                }
            }
        }

        // Compute axes based on latest repeats
        auto t = nowTP();
        // Axis1 (WS left/right): A/D
        double axis1 = 0;
        if (t - lastKey['a'] < releaseTimeout)
            axis1 = -holdPct;
        if (t - lastKey['d'] < releaseTimeout)
            axis1 = holdPct;
        // Axis2 (WS up/down): W/S
        double axis2 = 0;
        if (t - lastKey['w'] < releaseTimeout)
            axis2 = holdPct;
        if (t - lastKey['s'] < releaseTimeout)
            axis2 = -holdPct;
        // Axis3 (IJKL vertical): I/K
        double axis3 = 0;
        if (t - lastKey['i'] < releaseTimeout)
            axis3 = holdPct;
        if (t - lastKey['k'] < releaseTimeout)
            axis3 = -holdPct;
        // Axis4 (IJKL horizontal): J/L
        double axis4 = 0;
        if (t - lastKey['j'] < releaseTimeout)
            axis4 = -holdPct;
        if (t - lastKey['l'] < releaseTimeout)
            axis4 = holdPct;

        // Update shared axes which propagate to all controllers
        setSharedAxisPct(1, axis1);
        setSharedAxisPct(2, axis2);
        setSharedAxisPct(3, axis3);
        setSharedAxisPct(4, axis4);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void interactiveTerminal()
{
    // Helper to reapply interactive raw mode (blocking, VMIN=1, VTIME=0)
    auto applyInteractiveRawMode = []()
    {
        termios raw;
        if (tcgetattr(STDIN_FILENO, &raw) == 0)
        {
            raw.c_lflag &= ~(ICANON | ECHO | IEXTEN);
            raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
            raw.c_cflag |= (CS8);
            raw.c_cc[VMIN] = 1;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
        }
        // Ensure blocking reads
        int f = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (f == -1)
            f = 0;
        fcntl(STDIN_FILENO, F_SETFL, f & ~O_NONBLOCK);
    };

    // RAII for raw mode
    struct RawMode
    {
        termios orig;
        bool enabled{false};
        RawMode()
        {
            if (tcgetattr(STDIN_FILENO, &orig) == 0)
            {
                termios raw = orig;
                raw.c_lflag &= ~(ICANON | ECHO | IEXTEN);
                raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
                raw.c_cflag |= (CS8);
                raw.c_cc[VMIN] = 1;
                raw.c_cc[VTIME] = 0;
                if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == 0)
                {
                    enabled = true;
                }
            }
        }
        ~RawMode()
        {
            if (enabled)
            {
                tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig);
            }
        }
    } rawGuard;

    auto clearLine = []()
    {
        printf("\r\033[K");
        fflush(stdout);
    };
    auto printPrompt = []()
    {
        // Gray prompt, then switch back to white for input
        printf("\033[90mVEX Sim> \033[37m");
        fflush(stdout);
    };
    auto moveCursorTo = [](size_t target, size_t current)
    {
        if (target < current)
        {
            size_t n = current - target;
            for (size_t i = 0; i < n; ++i)
                printf("\033[D");
        }
        else if (target > current)
        {
            size_t n = target - current;
            for (size_t i = 0; i < n; ++i)
                printf("\033[C");
        }
        fflush(stdout);
    };
    auto redraw = [&](const std::string &buf, size_t cursorPos)
    {
        clearLine();
        printPrompt();
        fwrite(buf.data(), 1, buf.size(), stdout);
        fflush(stdout);
        // Move cursor back from end to cursorPos
        moveCursorTo(cursorPos, buf.size());
    };

    std::vector<std::string> history;
    int historyIndex = -1; // -1 means no selection; 0..n-1 indexes history

    std::string input;
    size_t cursor = 0;

    printPrompt();
    fflush(stdout);

    // Helper to process a single command (reused by watch)
    auto executeCommand = [&](const std::string &command)
    {
        if (command.substr(0, 4) == "exit")
        {
            printf("Exiting simulation terminal.\n");
            return -1; // signal to exit terminal loop
        }
        else if (command.substr(0, 5) == "clear")
        {
            // Clear the terminal screen
            printf("\033[2J\033[H"); // ANSI escape codes to clear screen and move cursor to home
        }
        else if (command.substr(0, 4) == "help")
        {
            printf("Available commands:\n");
            printf("  help - Show this help message\n");
            printf("  exit - Exit the simulation terminal\n");
            printf("  clear - Clear the simulation terminal\n");
            printf("  query [telemetry path] - Shows telemetry value\n");
            printf("  query all - Shows all telemetry values\n");
            printf("  watch [interval] [command] - Re-run command every interval seconds until 'q' is pressed\n");
            printf("  control start - Start keyboard control thread\n");
            printf("  control stop - Stop keyboard control thread\n");
        }
        else if (command == "control start")
        {
            // Start keyboard control thread
            if (g_controlRunning.load())
            {
                printf("Keyboard control already running.\n");
            }
            else
            {
                g_controlThread = std::thread(keyboardControlThread);
                g_controlThread.detach();
                printf("Started keyboard control. Press 'q' inside control to stop, or run 'control stop'.\n");
            }
        }
        else if (command == "control stop")
        {
            if (!g_controlRunning.load())
            {
                printf("Keyboard control is not running.\n");
            }
            else
            {
                g_controlRunning.store(false);
                printf("Stopping keyboard control...\n");
            }
        }
        else if (command.substr(0, 6) == "query ")
        {
            std::string path = command.substr(6);

            if (path == "all")
            {
                auto allValues = getAllTelemetryValues();
                for (const auto &entry : allValues)
                {
                    printf("Telemetry [%s] = %f\n", entry.first.c_str(), entry.second);
                }
            }
            else
            {
                double value;
                if (queryTelemetry(path, &value))
                {
                    printf("Telemetry [%s] = %f\n", path.c_str(), value);
                }
                else
                {
                    printf("Telemetry [%s] not found.\n", path.c_str());
                }
            }
        }
        else if (!command.empty())
        {
            printf("Unknown command: %s\n", command.c_str());
        }

        return 0;
    };

    while (true)
    {
        // If keyboard control is running, let it exclusively handle input (so 'q' works)
        if (g_controlRunning.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        unsigned char c = 0;
        ssize_t nread = read(STDIN_FILENO, &c, 1);
        if (nread <= 0)
        {
            // Input closed; exit
            printf("\n");
            break;
        }

        if (c == '\r' || c == '\n')
        {
            // Submit line
            printf("\n");
            std::string command = input;
            // Reset state for next command
            input.clear();
            cursor = 0;
            historyIndex = -1;
            if (!command.empty())
                history.push_back(command);

            // Process command and extended 'watch' feature
            if (command.substr(0, 6) == "watch ")
            {
                // Parse: watch [interval] [command]
                std::string rest = command.substr(6);
                // Optional flag support: --control to run keyboard control concurrently
                bool withControl = false;
                // Trim trailing spaces
                while (!rest.empty() && isspace(static_cast<unsigned char>(rest.back())))
                    rest.pop_back();
                // Detect trailing --control
                const std::string controlFlag = " --control";
                if (rest.size() >= controlFlag.size())
                {
                    // Check if rest ends with " --control" or "--control"
                    if (rest == "--control")
                    {
                        // Missing interval and command
                    }
                    else if (rest.rfind(controlFlag) != std::string::npos && rest.substr(rest.size() - controlFlag.size()) == controlFlag)
                    {
                        withControl = true;
                        rest.erase(rest.size() - controlFlag.size());
                        // Trim again
                        while (!rest.empty() && isspace(static_cast<unsigned char>(rest.back())))
                            rest.pop_back();
                    }
                }
                // Find first space separating interval and command
                size_t sp = rest.find(' ');
                if (sp == std::string::npos)
                {
                    printf("Usage: watch [interval] [command] [--control]\n");
                    printPrompt();
                    continue;
                }
                std::string intervalStr = rest.substr(0, sp);
                std::string innerCmd = rest.substr(sp + 1);

                char *endp = nullptr;
                double interval = strtod(intervalStr.c_str(), &endp);
                if (endp == intervalStr.c_str() || interval <= 0)
                {
                    printf("Invalid interval: %s\n", intervalStr.c_str());
                    printPrompt();
                    continue;
                }

                // Set stdin non-blocking
                int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
                if (flags == -1)
                    flags = 0;
                fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

                bool stop = false;
                bool startedControl = false;
                if (withControl && !g_controlRunning.load())
                {
                    // Start keyboard control thread concurrently
                    g_controlThread = std::thread(keyboardControlThread);
                    g_controlThread.detach();
                    startedControl = true;
                }
                while (!stop)
                {
                    // Clear screen and move cursor home
                    printf("\033[2J\033[H");
                    if (withControl)
                        printf("Watching: %s (interval: %.3fs) with control. Press 'q' to stop.\n", innerCmd.c_str(), interval);
                    else
                        printf("Watching: %s (interval: %.3fs). Press 'q' to stop.\n", innerCmd.c_str(), interval);
                    // Execute the command content below the header
                    int rc = executeCommand(innerCmd);
                    if (rc == -1)
                    {
                        // exit requested from inner command
                        stop = true;
                        break;
                    }

                    // Sleep in small slices while checking for 'q'
                    const int slices = 20;
                    double sliceDur = interval / slices;
                    for (int i = 0; i < slices && !stop; ++i)
                    {
                        // Check for 'q' key press
                        unsigned char k;
                        ssize_t r = read(STDIN_FILENO, &k, 1);
                        if (r > 0)
                        {
                            if (k == 'q' || k == 'Q')
                            {
                                stop = true;
                                break;
                            }
                            // Ignore other input while watching
                        }
                        std::this_thread::sleep_for(std::chrono::duration<double>(sliceDur));
                    }
                }

                // Restore blocking mode
                fcntl(STDIN_FILENO, F_SETFL, flags);
                printf("\nStopped watching.\n");
                // If we started control for this watch, stop it now
                if (startedControl && g_controlRunning.load())
                {
                    g_controlRunning.store(false);
                    printf("Stopping keyboard control started by watch...\n");
                    // Give control thread a moment to exit cleanly
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }

                // After watch (and possible control), reapply interactive raw mode
                applyInteractiveRawMode();
            }
            else
            {
                int rc = executeCommand(command);
                if (rc == -1)
                {
                    break;
                }
            }

            // Reprint prompt
            printPrompt();
            continue;
        }

        if (c == 0x7f || c == '\b')
        {
            // Backspace: delete char before cursor
            if (cursor > 0)
            {
                input.erase(cursor - 1, 1);
                cursor--;
                redraw(input, cursor);
            }
            continue;
        }

        if (c == '\x1b')
        {
            // Escape sequence: read next two bytes if available
            unsigned char seq[2] = {0, 0};
            if (read(STDIN_FILENO, &seq[0], 1) <= 0)
                continue;
            if (read(STDIN_FILENO, &seq[1], 1) <= 0)
                continue;
            if (seq[0] == '[')
            {
                switch (seq[1])
                {
                case 'A': // Up
                    if (!history.empty())
                    {
                        if (historyIndex < 0)
                            historyIndex = (int)history.size() - 1;
                        else
                            historyIndex = std::max(0, historyIndex - 1);
                        input = history[historyIndex];
                        cursor = input.size();
                        redraw(input, cursor);
                    }
                    break;
                case 'B': // Down
                    if (!history.empty())
                    {
                        if (historyIndex >= 0)
                        {
                            historyIndex++;
                            if (historyIndex >= (int)history.size())
                            {
                                historyIndex = -1;
                                input.clear();
                                cursor = 0;
                            }
                            else
                            {
                                input = history[historyIndex];
                                cursor = input.size();
                            }
                            redraw(input, cursor);
                        }
                    }
                    break;
                case 'C': // Right
                    if (cursor < input.size())
                    {
                        cursor++;
                        printf("\033[C");
                        fflush(stdout);
                    }
                    break;
                case 'D': // Left
                    if (cursor > 0)
                    {
                        cursor--;
                        printf("\033[D");
                        fflush(stdout);
                    }
                    break;
                default:
                    break;
                }
            }
            continue;
        }

        // Basic printable character insertion
        if (c >= 32 && c < 127)
        {
            input.insert(cursor, 1, (char)c);
            cursor++;
            redraw(input, cursor);
            continue;
        }
        // Ignore other control chars
    }
}

// Host-side memory telemetry: periodically publish RSS and virtual size
static void memoryTelemetryThread()
{
#ifdef __APPLE__
    while (true)
    {
        mach_task_basic_info info;
        mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
        kern_return_t kr = task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                                     reinterpret_cast<task_info_t>(&info), &count);
        if (kr == KERN_SUCCESS)
        {
            // resident_size is physical memory, virtual_size is total virtual
            const double toMB = 1024.0 * 1024.0;
            postTelemetry("system/memory/rss_mb", static_cast<double>(info.resident_size) / toMB);
            postTelemetry("system/memory/rss_limit_mb", static_cast<double>(RSS_MEMORY_CAP) / toMB);
            postTelemetry("system/memory/virtual_mb", static_cast<double>(info.virtual_size) / toMB);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
#else
    // On non-Apple hosts, just idle
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
#endif
}

// Central OOM handler for new allocations
static void onNewFailure()
{
#ifdef PANIC
    PANIC("Out of memory");
#else
    fprintf(stderr, "PANIC: Out of memory\n");
#endif
    std::abort();
}

// Enforce a hard memory cap on host (macOS) so allocations fail past limit
static bool enforceMemoryCapBytes(size_t bytes)
{
#ifdef __APPLE__
    struct rlimit lim;
    lim.rlim_cur = bytes;
    lim.rlim_max = bytes;
    if (setrlimit(RLIMIT_AS, &lim) != 0)
    {
        perror("setrlimit(RLIMIT_AS) failed");
        // Fallthrough to using watchdog
        // std::set_new_handler(onNewFailure);
        return false;
    }
#endif
    // std::set_new_handler(onNewFailure);
    return true;
}

// Fallback memory cap enforcement via periodic check (macOS only)
static void memoryCapWatchdogThread(size_t capBytes)
{
#ifdef __APPLE__
#if __has_include(<mach/mach.h>)
    const double toMB = 1024.0 * 1024.0;
    while (true)
    {
        mach_task_basic_info info;
        mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
        kern_return_t kr = task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                                     reinterpret_cast<task_info_t>(&info), &count);
        if (kr == KERN_SUCCESS)
        {
            // Use resident_size (RSS) as cap target
            if (info.resident_size > capBytes)
            {
#ifdef PANIC
                PANIC("Out of memory (watchdog). rss=%.2f MB cap=%.2f MB",
                      info.resident_size / toMB, capBytes / toMB);
#else
                fprintf(stderr, "PANIC: Out of memory (watchdog). rss=%.2f MB cap=%.2f MB\n",
                        info.resident_size / toMB, capBytes / toMB);
#endif
                std::abort();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
#else
    (void)capBytes;
    while (true)
        std::this_thread::sleep_for(std::chrono::seconds(1));
#endif
#else
    (void)capBytes;
    while (true)
        std::this_thread::sleep_for(std::chrono::seconds(1));
#endif
}

// Serialize all telemetry values to a compact JSON string
static std::string telemetryToJson()
{
    std::lock_guard<std::mutex> lock(getTelemetryMapMutex());
    const auto &map = getTelemetryMap();
    std::string out;
    out.reserve(1024);
    out.push_back('{');
    bool first = true;
    for (const auto &kv : map)
    {
        if (!first)
            out.push_back(',');
        first = false;
        // Minimal JSON escaping for keys: escape backslash and quotes
        std::string keyEsc;
        keyEsc.reserve(kv.first.size() + 8);
        for (char ch : kv.first)
        {
            if (ch == '"' || ch == '\\')
            {
                keyEsc.push_back('\\');
                keyEsc.push_back(ch);
            }
            else
            {
                keyEsc.push_back(ch);
            }
        }
        out.append("\"");
        out.append(keyEsc);
        out.append("\":");
        // Render double with default formatting
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "%g", kv.second);
        if (n > 0)
            out.append(buf, static_cast<size_t>(n));
    }
    out.push_back('}');
    return out;
}

// uWebSockets server thread exposing telemetry over HTTP
static void uwsServerThread()
{
    try
    {
        // Simple HTTP + WebSocket API:
        //  - GET /telemetry -> JSON of all telemetry
        //  - GET /query?path=... -> single value or 404
        //  - WS /telemetry -> subscribe to live telemetry broadcasts (JSON) every 20ms

        // Create app instance explicitly so we can publish from a background thread
        auto app = std::make_shared<uWS::App>();

        // Helper: content type by extension
        auto contentTypeFor = [](const std::string &path) -> const char *
        {
            auto extpos = path.find_last_of('.');
            std::string ext = (extpos == std::string::npos) ? std::string() : path.substr(extpos + 1);
            if (ext == "html" || ext == "htm")
                return "text/html";
            if (ext == "css")
                return "text/css";
            if (ext == "js")
                return "application/javascript";
            if (ext == "json")
                return "application/json";
            if (ext == "png")
                return "image/png";
            if (ext == "jpg" || ext == "jpeg")
                return "image/jpeg";
            if (ext == "svg")
                return "image/svg+xml";
            if (ext == "ico")
                return "image/x-icon";
            if (ext == "txt")
                return "text/plain";
            return "application/octet-stream";
        };

        // Base directory for static files
        const std::filesystem::path staticBase = std::filesystem::path("public");

        // HTTP endpoints
        app->get("/telemetry", [](auto *res, auto *req)
                 {
            res->writeHeader("Content-Type", "application/json");
            res->end(telemetryToJson()); });

        app->get("/query", [](auto *res, auto *req)
                 {
            auto path = req->getQuery("path");
            if (path.empty())
            {
                res->writeStatus("400 Bad Request")->end("missing 'path' query param");
                return;
            }
            double v;
            if (queryTelemetry(std::string(path), &v))
            {
                char buf[64];
                int n = snprintf(buf, sizeof(buf), "%g", v);
                res->writeHeader("Content-Type", "text/plain");
                res->end(std::string(buf, (n > 0) ? (size_t)n : 0));
            }
            else
            {
                res->writeStatus("404 Not Found")->end("not found");
            } });

        // WebSocket route for telemetry streaming
        struct PerSocketData
        {
        };
        app->ws<PerSocketData>("/telemetry",
                               {// On open, subscribe client to the "telemetry" topic
                                .open = [](auto *ws)
                                { ws->subscribe("telemetry"); },
                                // Handle incoming control packets from clients
                                .message = [](auto *ws, std::string_view msg, uWS::OpCode code)
                                {
                                    (void)ws;
                                    // Accept TEXT or BINARY; assume UTF-8 JSON
                                    try {
                                        std::string json(msg.data(), msg.size());
                                        processIncomingPacket(json);
                                    } catch (...) {
                                        // ignore malformed packet
                                    } },
                                .close = [](auto *ws, int code, std::string_view msg)
                                {
                                    (void)ws;
                                    (void)code;
                                    (void)msg; }});

        // Static file server for all remaining GET requests under /*
        app->get("/*", [staticBase, contentTypeFor](auto *res, auto *req)
                 {
            // Map URL to file path under public/
            std::string urlPath = std::string(req->getUrl());
            if (urlPath.empty() || urlPath == "/")
                urlPath = "/index.html";

            // Prevent path traversal: normalize and ensure within staticBase
            std::filesystem::path requested = staticBase / urlPath.substr(1); // drop leading '/'
            std::error_code ec;
            std::filesystem::path canonicalBase = std::filesystem::weakly_canonical(staticBase, ec);
            std::filesystem::path canonicalRequested = std::filesystem::weakly_canonical(requested, ec);
            if (!ec)
            {
                // If path is a directory, serve its index.html
                if (std::filesystem::is_directory(canonicalRequested))
                {
                    canonicalRequested /= "index.html";
                }

                // Ensure requested is under base
                auto baseStr = canonicalBase.string();
                auto reqStr = canonicalRequested.string();
                if (reqStr.compare(0, baseStr.size(), baseStr) != 0)
                {
                    res->writeStatus("403 Forbidden")->end("forbidden");
                    return;
                }

                // Read file contents
                std::ifstream file(canonicalRequested, std::ios::binary);
                if (!file.good())
                {
                    res->writeStatus("404 Not Found")->end("not found");
                    return;
                }
                std::ostringstream buffer;
                buffer << file.rdbuf();
                std::string body = buffer.str();

                res->writeHeader("Content-Type", contentTypeFor(reqStr))
                   ->writeHeader("Cache-Control", "no-cache")
                   ->end(std::move(body));
                return;
            }
            // If canonicalization failed, return 404
            res->writeStatus("404 Not Found")->end("not found"); });

        // Fallback for unknown endpoints
        app->any("/*", [](auto *res, auto *req)
                 { res->writeStatus("404 Not Found")->end("endpoint not found"); });

        // Listen on port 9001
        app->listen(9001, [](auto *token)
                    {
            if (token)
            {
                printf("uWS server listening on http://localhost:9001\n");
            }
            else
            {
                fprintf(stderr, "Failed to listen on port 9001 (uWS)\n");
            } });

        // Obtain the uWS event loop for this thread
        uWS::Loop *loop = uWS::Loop::get();

        // Publisher thread: schedule publish on the uWS loop to remain thread-safe
        std::thread publisher([app, loop]()
                              {
            try
            {
                while (true)
                {
                    // Defer publish to the uWS thread
                    loop->defer([app]()
                                {
                        app->publish("telemetry", telemetryToJson(), uWS::OpCode::TEXT);
                    });
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
            catch (...)
            {
                fprintf(stderr, "uWS telemetry publisher crashed\n");
            } });
        publisher.detach();

        // Run server loop (blocking)
        app->run();
    }
    catch (...)
    {
        fprintf(stderr, "uWS server thread crashed\n");
    }
}

void vex::competition::drivercontrol(void (*drivercontrolFunction)())
{
    // For host/macOS simulation, enforce ~100 MB cap and start memory telemetry
    const size_t capBytes = RSS_MEMORY_CAP;
    bool capped = enforceMemoryCapBytes(capBytes);

    std::thread simThread(simulationThread);
    simThread.detach();

    std::thread terminalThread(interactiveTerminal);
    terminalThread.detach();

    std::thread memThread(memoryTelemetryThread);
    memThread.detach();

    // Start uWebSockets server for telemetry/queries
    std::thread uwsThread(uwsServerThread);
    uwsThread.detach();

    if (!capped)
    {
        // Start watchdog if OS cap failed
        std::thread watchdogThread(memoryCapWatchdogThread, capBytes);
        watchdogThread.detach();
    }
}

int getSimulationAlliance()
{
    return 0;
}

#endif