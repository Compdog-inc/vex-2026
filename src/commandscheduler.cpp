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

void CommandScheduler::removeEndedCommands()
{
    for (Command *command : commands_to_remove)
    {
        scheduled_commands.erase(std::remove(scheduled_commands.begin(), scheduled_commands.end(), command), scheduled_commands.end());
        if (command->onHeap())
        {
            delete command;
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

FunctionalCommand::FunctionalCommand(const std::function<void()> &onInitialize,
                                     const std::function<void()> &onExecute,
                                     const std::function<bool()> &onIsFinished,
                                     const std::function<void(bool)> &onEnd,
                                     const std::vector<Subsystem *> &requirements)
    : onInitialize(onInitialize),
      onExecute(onExecute),
      onIsFinished(onIsFinished),
      onEnd(onEnd)
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
                                       const std::function<bool()> &condition)
    : onTrue(onTrue),
      onFalse(onFalse),
      condition(condition),
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

DeferredCommand::DeferredCommand(const std::function<Command *()> &commandSupplier,
                                 const std::vector<Subsystem *> &requirements)
    : commandSupplier(commandSupplier),
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
