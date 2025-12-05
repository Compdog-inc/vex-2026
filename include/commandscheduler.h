#ifndef __COMMAND_SCHEDULER_H__
#define __COMMAND_SCHEDULER_H__

#include <vector>
#include <unordered_set>
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
    FunctionalCommand(const std::function<void()> &onInitialize,
                      const std::function<void()> &onExecute,
                      const std::function<bool()> &onIsFinished,
                      const std::function<void(bool)> &onEnd,
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
    InstantCommand(const std::function<void()> &toRun,
                   const std::vector<Subsystem *> &requirements = {})
        : FunctionalCommand(toRun, []() {}, []()
                            { return true; }, [](bool) {}, requirements)
    {
    }
};

class ConditionalCommand : public Command
{
public:
    ConditionalCommand(
        Command *onTrue,
        Command *onFalse,
        const std::function<bool()> &condition);

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
    DeferredCommand(const std::function<Command *()> &commandSupplier,
                    const std::vector<Subsystem *> &requirements);

    void initialize() override;
    void execute() override;
    bool isFinished() override;
    void end(bool interrupted) override;

private:
    std::function<Command *()> commandSupplier;
    Command *scheduledCommand = nullptr;
};

class Commands
{
public:
    /**
     * Runs the command supplied by the supplier.
     *
     * @param commandSupplier the command supplier
     * @param requirements the set of requirements for this command
     * @return the command
     * @see DeferredCommand
     */
    static Command *defer(const std::function<Command *()> &commandSupplier,
                          const std::vector<Subsystem *> &requirements)
    {
        return new DeferredCommand(commandSupplier, requirements);
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
        return new SequentialCommandGroup(commands);
    }
};

#endif