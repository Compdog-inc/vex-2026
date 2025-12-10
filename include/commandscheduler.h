#ifndef __COMMAND_SCHEDULER_H__
#define __COMMAND_SCHEDULER_H__

#include "vex.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <functional>

class Command
{
public:
    virtual ~Command() = default;
    virtual void initialize() {}
    virtual void execute() {}

    virtual bool isFinished() = 0;
    virtual void end(bool interrupted) {}

    virtual bool onHeap() { return internal_onHeap; }
    void setOnHeap(bool onHeap) { internal_onHeap = onHeap; }

    void addRequirement(class Subsystem *subsystem);

    std::unordered_set<class Subsystem *> getRequirements()
    {
        return requirements;
    }

    Command *withTimeout(double seconds);
    Command *andThen(Command *next);
    Command *andThen(std::function<void()> toRun);
    Command *andThen(const std::vector<Command *> &nextCommands);
    Command *withDeadline(Command *deadline);
    Command *alongWith(const std::vector<Command *> &commands);
    Command *raceWith(const std::vector<Command *> &commands);

private:
    bool internal_onHeap = false;
    std::unordered_set<Subsystem *> requirements{};
};

class Subsystem
{
public:
    virtual void periodic() {}

    void setDefaultCommand(Command *command);
    Command *getDefaultCommand();
    void setCurrentCommand(Command *command);
    Command *getCurrentCommand();

private:
    Command *defaultCommand = nullptr;
    Command *currentCommand = nullptr;
};

class CommandScheduler
{
public:
    CommandScheduler();

    static CommandScheduler *getInstance();

    void run();
    void schedule(Command *command);

    void registerSubsystem(Subsystem *subsystem);

    void cancel(Command *command, bool interrupted);
    void removeEndedCommands();

private:
    std::vector<Subsystem *> subsystems;
    std::vector<Command *> scheduled_commands;
    std::vector<Command *> commands_to_remove;
};

class SequentialCommandGroup : public Command
{
public:
    SequentialCommandGroup(const std::vector<Command *> &commands);

    void addCommands(const std::vector<Command *> &commands);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    std::vector<Command *> commands;
    size_t currentCommandIndex = -1;
};

class FunctionalCommand : public Command
{
public:
    FunctionalCommand(std::function<void()> onInitialize,
                      std::function<void()> onExecute,
                      std::function<bool()> onIsFinished,
                      std::function<void(bool)> onEnd,
                      const std::vector<Subsystem *> &requirements = {});

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    std::function<void()> onInitialize;
    std::function<void()> onExecute;
    std::function<bool()> onIsFinished;
    std::function<void(bool)> onEnd;
};

class InstantCommand : public FunctionalCommand
{
public:
    InstantCommand(std::function<void()> toRun,
                   const std::vector<Subsystem *> &requirements = {})
        : FunctionalCommand(toRun, []() {}, []()
                            { return true; }, [](bool) {}, requirements)
    {
    }

    InstantCommand()
        : InstantCommand([]() {}) {}
};

class ConditionalCommand : public Command
{
public:
    ConditionalCommand(
        Command *onTrue,
        Command *onFalse,
        std::function<bool()> condition);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    Command *onTrue;
    Command *onFalse;
    std::function<bool()> condition;
    Command *selectedCommand = nullptr;
};

class DeferredCommand : public Command
{
public:
    DeferredCommand(std::function<Command *()> commandSupplier,
                    const std::vector<Subsystem *> &requirements);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    std::function<Command *()> commandSupplier;
    Command *scheduledCommand = nullptr;
};

class WaitCommand : public Command
{
public:
    WaitCommand(double seconds);
    WaitCommand(double value, vex::timeUnits units);

    void initialize() override;
    bool isFinished() override;

private:
    vex::timer timer;
    double duration;
    vex::timeUnits timeUnit;
};

class WaitUntilCommand : public Command
{
public:
    WaitUntilCommand(std::function<bool()> condition);

    bool isFinished() override;

private:
    std::function<bool()> condition;
};

class ParallelCommandGroup : public Command
{
public:
    ParallelCommandGroup(const std::vector<Command *> &commands);

    void addCommands(const std::vector<Command *> &commands);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    std::unordered_map<Command *, bool> commands;
};

class ParallelRaceGroup : public Command
{
public:
    ParallelRaceGroup(const std::vector<Command *> &commands);

    void addCommands(const std::vector<Command *> &commands);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    std::unordered_set<Command *> commands;
    bool finished = true;
};

class ParallelDeadlineGroup : public Command
{
public:
    ParallelDeadlineGroup(Command *deadline, const std::vector<Command *> &commands);

    void setDeadline(Command *deadline);

    void addCommands(const std::vector<Command *> &commands);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    std::unordered_map<Command *, bool> commands;
    bool finished = true;
    Command *deadline;
};

class Commands
{
public:
    /**
     * Constructs a command that does nothing, finishing immediately.
     *
     * @return the command
     */
    static Command *none()
    {
        InstantCommand *cmd = new InstantCommand();
        cmd->setOnHeap(true);
        return cmd;
    }

    /**
     * Constructs a command that runs an action once and finishes.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see InstantCommand
     */
    static Command *runOnce(std::function<void()> toRun,
                            const std::vector<Subsystem *> &requirements = {})
    {
        InstantCommand *cmd = new InstantCommand(toRun, requirements);
        cmd->setOnHeap(true);
        return cmd;
    }

    /**
     * Runs the command supplied by the supplier.
     *
     * @param commandSupplier the command supplier
     * @param requirements the set of requirements for this command
     * @return the command
     * @see DeferredCommand
     */
    static Command *defer(std::function<Command *()> commandSupplier,
                          const std::vector<Subsystem *> &requirements)
    {
        DeferredCommand *cmd = new DeferredCommand(commandSupplier, requirements);
        cmd->setOnHeap(true);
        return cmd;
    }

    /**
     * Runs a group of commands in series, one after the other.
     *
     * @param commands the commands to include
     * @return the command group
     * @see SequentialCommandGroup
     */
    static Command *sequence(const std::vector<Command *> &commands)
    {
        SequentialCommandGroup *cmd = new SequentialCommandGroup(commands);
        cmd->setOnHeap(true);
        return cmd;
    }

    /**
     * Runs a group of commands at the same time. Ends once all commands in the group finish.
     *
     * @param commands the commands to include
     * @return the command
     * @see ParallelCommandGroup
     */
    static Command *parallel(const std::vector<Command *> &commands)
    {
        ParallelCommandGroup *cmd = new ParallelCommandGroup(commands);
        cmd->setOnHeap(true);
        return cmd;
    }

    /**
     * Runs a group of commands at the same time. Ends once any command in the group finishes, and
     * cancels the others.
     *
     * @param commands the commands to include
     * @return the command group
     * @see ParallelRaceGroup
     */
    static Command *race(const std::vector<Command *> &commands)
    {
        ParallelRaceGroup *cmd = new ParallelRaceGroup(commands);
        cmd->setOnHeap(true);
        return cmd;
    }

    /**
     * Runs a group of commands at the same time. Ends once a specific command finishes, and cancels
     * the others.
     *
     * @param deadline the deadline command
     * @param otherCommands the other commands to include
     * @return the command group
     * @see ParallelDeadlineGroup
     * @throws IllegalArgumentException if the deadline command is also in the otherCommands argument
     */
    static Command *deadline(Command *deadline, const std::vector<Command *> &otherCommands)
    {
        ParallelDeadlineGroup *cmd = new ParallelDeadlineGroup(deadline, otherCommands);
        cmd->setOnHeap(true);
        return cmd;
    }

    static Command *wait(double seconds)
    {
        WaitCommand *cmd = new WaitCommand(seconds);
        cmd->setOnHeap(true);
        return cmd;
    }

    static Command *waitUntilCondition(std::function<bool()> condition)
    {
        WaitUntilCommand *cmd = new WaitUntilCommand(condition);
        cmd->setOnHeap(true);
        return cmd;
    }
};

#endif