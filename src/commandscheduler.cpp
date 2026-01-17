#include "commandscheduler.h"
#include <algorithm>
#include "rmath.h"

static CommandScheduler instance{};

void Subsystem::setDefaultCommand(Command *command)
{
    defaultCommand = command;
    CommandScheduler::getInstance()->schedule(command);
}

Command *Subsystem::getDefaultCommand()
{
    return defaultCommand;
}

void Subsystem::setCurrentCommand(Command *command)
{
    currentCommand = command;
}

Command *Subsystem::getCurrentCommand()
{
    return currentCommand;
}

Command *Command::withTimeout(double seconds)
{
    WaitCommand *waitCmd = new WaitCommand(seconds);
    waitCmd->setOnHeap(true);
    return raceWith({waitCmd});
}

Command *Command::andThen(Command *next)
{
    return andThen(std::vector<Command *>{next});
}

Command *Command::andThen(std::function<void()> toRun)
{
    InstantCommand *instantCmd = new InstantCommand(toRun);
    instantCmd->setOnHeap(true);
    return andThen(instantCmd);
}

Command *Command::andThen(const std::vector<Command *> &nextCommands)
{
    SequentialCommandGroup *seq = new SequentialCommandGroup({this});
    seq->addCommands(nextCommands);
    seq->setOnHeap(true);
    return seq;
}

Command *Command::withDeadline(Command *deadline)
{
    ParallelDeadlineGroup *pdg = new ParallelDeadlineGroup(deadline, {this});
    pdg->setOnHeap(true);
    return pdg;
}

Command *Command::alongWith(const std::vector<Command *> &commands)
{
    ParallelCommandGroup *pcg = new ParallelCommandGroup({this});
    pcg->addCommands(commands);
    pcg->setOnHeap(true);
    return pcg;
}

Command *Command::raceWith(const std::vector<Command *> &commands)
{
    ParallelRaceGroup *prg = new ParallelRaceGroup({this});
    prg->addCommands(commands);
    prg->setOnHeap(true);
    return prg;
}

CommandScheduler::CommandScheduler()
    : subsystems(), scheduled_commands()
{
}

CommandScheduler *CommandScheduler::getInstance()
{
    return &instance;
}

void CommandScheduler::run()
{
    for (Subsystem *subsystem : subsystems)
    {
        subsystem->periodic();
    }

    for (Command *command : scheduled_commands)
    {
        command->execute();
        if (command->isFinished())
        {
            cancel(command, false);
        }
    }

    removeEndedCommands();
}

void CommandScheduler::cancel(Command *command, bool interrupted)
{
    for (Subsystem *subsystem : command->getRequirements())
    {
        if (subsystem->getCurrentCommand() == command)
        {
            subsystem->setCurrentCommand(nullptr);
            if (subsystem->getDefaultCommand() != nullptr && !interrupted)
            {
                schedule(subsystem->getDefaultCommand());
            }
        }
    }

    commands_to_remove.push_back(command);
    command->end(interrupted);
}

void CommandScheduler::cancelAll()
{
    for (Command *command : scheduled_commands)
    {
        cancel(command, true);
    }
}

void CommandScheduler::removeEndedCommands()
{
    for (Command *command : commands_to_remove)
    {
        scheduled_commands.erase(std::remove(scheduled_commands.begin(), scheduled_commands.end(), command), scheduled_commands.end());
        if (command->onHeap())
        {
            // delete command;
        }
    }

    commands_to_remove.clear();
}

void CommandScheduler::schedule(Command *command)
{
    for (Command *scheduledCommand : scheduled_commands)
    {
        if (scheduledCommand == command)
        {
            return; // Command is already scheduled
        }

        if (MathUtil::intersects(scheduledCommand->getRequirements(), command->getRequirements()))
        {
            // Interrupt the conflicting command
            cancel(scheduledCommand, true);
        }
    }

    removeEndedCommands();

    scheduled_commands.push_back(command);
    for (Subsystem *subsystem : command->getRequirements())
    {
        subsystem->setCurrentCommand(command);
    }
    command->initialize();
}

void CommandScheduler::registerSubsystem(Subsystem *subsystem)
{
    subsystems.push_back(subsystem);
}

void Command::addRequirement(Subsystem *subsystem)
{
    requirements.insert(subsystem);
}

SequentialCommandGroup::SequentialCommandGroup(const std::vector<Command *> &commands) : commands()
{
    addCommands(commands);
}

void SequentialCommandGroup::addCommands(const std::vector<Command *> &commands)
{
    for (Command *command : commands)
    {
        this->commands.push_back(command);
        for (Subsystem *subsystem : command->getRequirements())
        {
            addRequirement(subsystem);
        }
    }
}

void SequentialCommandGroup::initialize()
{
    currentCommandIndex = 0;
    if (!commands.empty())
    {
        commands[currentCommandIndex]->initialize();
    }
}

void SequentialCommandGroup::execute()
{
    if (commands.empty())
        return;

    if (currentCommandIndex < commands.size())
    {
        Command *currentCommand = commands[currentCommandIndex];
        currentCommand->execute();
        if (currentCommand->isFinished())
        {
            currentCommand->end(false);
            currentCommandIndex++;
            if (currentCommandIndex < commands.size())
            {
                commands[currentCommandIndex]->initialize();
            }
        }
    }
}

bool SequentialCommandGroup::isFinished()
{
    return currentCommandIndex >= commands.size();
}

void SequentialCommandGroup::end(bool interrupted)
{
    if (interrupted && !commands.empty() && currentCommandIndex > -1 && currentCommandIndex < commands.size())
    {
        commands[currentCommandIndex]->end(true);
    }
    currentCommandIndex = -1;
}

FunctionalCommand::FunctionalCommand(std::function<void()> onInitialize,
                                     std::function<void()> onExecute,
                                     std::function<bool()> onIsFinished,
                                     std::function<void(bool)> onEnd,
                                     const std::vector<Subsystem *> &requirements)
    : onInitialize(std::move(onInitialize)),
      onExecute(std::move(onExecute)),
      onIsFinished(std::move(onIsFinished)),
      onEnd(std::move(onEnd))
{
    for (Subsystem *subsystem : requirements)
    {
        addRequirement(subsystem);
    }
}

void FunctionalCommand::initialize()
{
    if (onInitialize)
    {
        onInitialize();
    }
}

void FunctionalCommand::execute()
{
    if (onExecute)
    {
        onExecute();
    }
}

bool FunctionalCommand::isFinished()
{
    if (onIsFinished)
    {
        return onIsFinished();
    }
    return true;
}

void FunctionalCommand::end(bool interrupted)
{
    if (onEnd)
    {
        onEnd(interrupted);
    }
}

ConditionalCommand::ConditionalCommand(Command *onTrue,
                                       Command *onFalse,
                                       std::function<bool()> condition)
    : onTrue(onTrue),
      onFalse(onFalse),
      condition(std::move(condition)),
      selectedCommand(nullptr)
{
    for (Subsystem *subsystem : onTrue->getRequirements())
    {
        addRequirement(subsystem);
    }
    for (Subsystem *subsystem : onFalse->getRequirements())
    {
        addRequirement(subsystem);
    }
}

void ConditionalCommand::initialize()
{
    if (condition())
    {
        selectedCommand = onTrue;
    }
    else
    {
        selectedCommand = onFalse;
    }

    if (selectedCommand)
    {
        selectedCommand->initialize();
    }
}

void ConditionalCommand::execute()
{
    if (selectedCommand)
    {
        selectedCommand->execute();
    }
}

bool ConditionalCommand::isFinished()
{
    if (selectedCommand)
    {
        return selectedCommand->isFinished();
    }
    return true;
}

void ConditionalCommand::end(bool interrupted)
{
    if (selectedCommand)
    {
        selectedCommand->end(interrupted);
    }
}

DeferredCommand::DeferredCommand(std::function<Command *()> commandSupplier,
                                 const std::vector<Subsystem *> &requirements)
    : commandSupplier(std::move(commandSupplier)),
      scheduledCommand(nullptr)
{
    for (Subsystem *subsystem : requirements)
    {
        addRequirement(subsystem);
    }
}

void DeferredCommand::initialize()
{
    scheduledCommand = commandSupplier();
    if (scheduledCommand)
    {
        scheduledCommand->initialize();
    }
}

void DeferredCommand::execute()
{
    if (scheduledCommand)
    {
        scheduledCommand->execute();
    }
}

bool DeferredCommand::isFinished()
{
    if (scheduledCommand)
    {
        return scheduledCommand->isFinished();
    }
    return true;
}

void DeferredCommand::end(bool interrupted)
{
    if (scheduledCommand)
    {
        scheduledCommand->end(interrupted);
        scheduledCommand = nullptr;
    }
}

WaitCommand::WaitCommand(double durationSeconds)
    : WaitCommand(durationSeconds, vex::timeUnits::sec)
{
}

WaitCommand::WaitCommand(double value, vex::timeUnits units)
    : timer(),
      duration(value),
      timeUnit(units)
{
}

void WaitCommand::initialize()
{
    timer.reset();
}

bool WaitCommand::isFinished()
{
    double elapsedTime = timer.time(timeUnit);
    return elapsedTime >= duration;
}

WaitUntilCommand::WaitUntilCommand(std::function<bool()> condition)
    : condition(std::move(condition))
{
}

bool WaitUntilCommand::isFinished()
{
    return condition();
}

ParallelCommandGroup::ParallelCommandGroup(const std::vector<Command *> &commands)
    : commands()
{
    addCommands(commands);
}

void ParallelCommandGroup::addCommands(const std::vector<Command *> &commands)
{
    for (Command *command : commands)
    {
        this->commands[command] = false;
        for (Subsystem *subsystem : command->getRequirements())
        {
            addRequirement(subsystem);
        }
    }
}

void ParallelCommandGroup::initialize()
{
    for (auto &pair : commands)
    {
        Command *command = pair.first;
        command->initialize();
        pair.second = true; // Running
    }
}

void ParallelCommandGroup::execute()
{
    for (auto &pair : commands)
    {
        Command *command = pair.first;
        bool &isRunning = pair.second;

        if (isRunning)
        {
            command->execute();
            if (command->isFinished())
            {
                command->end(false);
                isRunning = false; // Mark as finished
            }
        }
    }
}

bool ParallelCommandGroup::isFinished()
{
    for (const auto &pair : commands)
    {
        if (pair.second) // If any command is still running
        {
            return false;
        }
    }
    return true;
}

void ParallelCommandGroup::end(bool interrupted)
{
    for (auto &pair : commands)
    {
        Command *command = pair.first;
        bool &isRunning = pair.second;

        if (isRunning)
        {
            command->end(interrupted);
            isRunning = false;
        }
    }
}

ParallelRaceGroup::ParallelRaceGroup(const std::vector<Command *> &commands) : commands()
{
    addCommands(commands);
}

void ParallelRaceGroup::addCommands(const std::vector<Command *> &commands)
{
    for (Command *command : commands)
    {
        this->commands.insert(command);
        for (Subsystem *subsystem : command->getRequirements())
        {
            addRequirement(subsystem);
        }
    }
}

void ParallelRaceGroup::initialize()
{
    finished = false;
    for (Command *command : commands)
    {
        command->initialize();
    }
}

void ParallelRaceGroup::execute()
{
    for (Command *command : commands)
    {
        command->execute();
        if (command->isFinished())
        {
            finished = true;
        }
    }
}

void ParallelRaceGroup::end(bool interrupted)
{
    for (Command *command : commands)
    {
        command->end(!command->isFinished());
    }
}

bool ParallelRaceGroup::isFinished()
{
    return finished;
}

ParallelDeadlineGroup::ParallelDeadlineGroup(Command *deadline, const std::vector<Command *> &commands)
    : commands()
{
    setDeadline(deadline);
    addCommands(commands);
}

void ParallelDeadlineGroup::setDeadline(Command *deadline)
{
    bool isAlreadyDeadline = deadline == this->deadline;
    if (isAlreadyDeadline)
    {
        return;
    }

    addCommands({deadline});
    this->deadline = deadline;
}

void ParallelDeadlineGroup::addCommands(const std::vector<Command *> &commands)
{
    for (Command *command : commands)
    {
        this->commands[command] = false;
        for (Subsystem *subsystem : command->getRequirements())
        {
            addRequirement(subsystem);
        }
    }
}

void ParallelDeadlineGroup::initialize()
{
    for (auto &pair : commands)
    {
        Command *command = pair.first;
        command->initialize();
        pair.second = true; // Running
    }
    finished = false;
}

void ParallelDeadlineGroup::execute()
{
    for (auto &pair : commands)
    {
        Command *command = pair.first;
        bool &isRunning = pair.second;

        if (isRunning)
        {
            command->execute();
            if (command->isFinished())
            {
                command->end(false);
                isRunning = false; // Mark as finished
                if (command == deadline)
                {
                    finished = true;
                }
            }
        }
    }
}

void ParallelDeadlineGroup::end(bool interrupted)
{
    for (auto &pair : commands)
    {
        Command *command = pair.first;
        bool &isRunning = pair.second;

        if (isRunning)
        {
            command->end(true);
        }
    }
}

bool ParallelDeadlineGroup::isFinished()
{
    return finished;
}