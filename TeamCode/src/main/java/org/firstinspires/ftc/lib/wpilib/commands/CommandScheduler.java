// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.commands;

import static org.firstinspires.ftc.lib.wpilib.commands.Command.InterruptionBehavior;
import static org.firstinspires.ftc.lib.wpilib.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.WeakHashMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import org.firstinspires.ftc.lib.wpilib.event.EventLoop;

/**
 * The scheduler responsible for running {@link Command}s. A Command-based robot should call {@link
 * CommandScheduler#run()} on the singleton instance in its periodic block in order to run commands
 * synchronously from the main loop. Subsystems should be registered with the scheduler using {@link
 * CommandScheduler#registerSubsystem(Subsystem...)} in order for their {@link Subsystem#periodic()}
 * methods to be called and for their default commands to be scheduled.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public final class CommandScheduler {
  /** The Singleton Instance. */
  private static CommandScheduler instance;

  public static void initScheduler() {
    instance = new CommandScheduler();
  }

  public static void clearScheduler() {
    instance = null;
  }

  /**
   * Returns the Scheduler instance.
   *
   * @return the instance
   */
  public static CommandScheduler getInstance() {
    return instance;
  }

  private static final Optional<Command> kNoInterruptor = Optional.empty();

  private final Map<Command, Exception> m_composedCommands = new WeakHashMap<>();

  // A set of the currently-running commands.
  private final Set<Command> m_scheduledCommands = new LinkedHashSet<>();

  // A map from required subsystems to their requiring commands. Also used as a set of the
  // currently-required subsystems.
  private final Map<Subsystem, Command> m_requirements = new LinkedHashMap<>();

  // A map from subsystems registered with the scheduler to their default commands.  Also used
  // as a list of currently-registered subsystems.
  private final Map<Subsystem, Command> m_subsystems = new LinkedHashMap<>();

  private final EventLoop m_defaultButtonLoop = new EventLoop();
  // The set of currently-registered buttons that will be polled every iteration.
  private EventLoop m_activeButtonLoop = m_defaultButtonLoop;

  private boolean m_disabled;

  // Lists of user-supplied actions to be executed on scheduling events for every command.
  private final List<Consumer<Command>> m_initActions = new ArrayList<>();
  private final List<Consumer<Command>> m_executeActions = new ArrayList<>();
  private final List<BiConsumer<Command, Optional<Command>>> m_interruptActions = new ArrayList<>();
  private final List<Consumer<Command>> m_finishActions = new ArrayList<>();

  // Flag and queues for avoiding ConcurrentModificationException if commands are
  // scheduled/canceled during run
  private boolean m_inRunLoop;
  private final Set<Command> m_toSchedule = new LinkedHashSet<>();
  private final List<Command> m_toCancelCommands = new ArrayList<>();
  private final List<Optional<Command>> m_toCancelInterruptors = new ArrayList<>();
  private final Set<Command> m_endingCommands = new LinkedHashSet<>();

  /**
   * Get the default button poll.
   *
   * @return a reference to the default {@link EventLoop} object polling buttons.
   */
  public EventLoop getDefaultButtonLoop() {
    return m_defaultButtonLoop;
  }

  /**
   * Get the active button poll.
   *
   * @return a reference to the current {@link EventLoop} object polling buttons.
   */
  public EventLoop getActiveButtonLoop() {
    return m_activeButtonLoop;
  }

  /**
   * Replace the button poll with another one.
   *
   * @param loop the new button polling loop object.
   */
  public void setActiveButtonLoop(EventLoop loop) {
    m_activeButtonLoop =
        requireNonNullParam(loop, "loop", "CommandScheduler" + ".replaceButtonEventLoop");
  }

  /**
   * Initializes a given command, adds its requirements to the list, and performs the init actions.
   *
   * @param command The command to initialize
   * @param requirements The command requirements
   */
  private void initCommand(Command command, Set<Subsystem> requirements) {
    m_scheduledCommands.add(command);
    for (Subsystem requirement : requirements) {
      m_requirements.put(requirement, command);
    }
    command.initialize();
    for (Consumer<Command> action : m_initActions) {
      action.accept(command);
    }

    //    m_watchdog.addEpoch(command.getName() + ".initialize()");
  }

  /**
   * Schedules a command for execution. Does nothing if the command is already scheduled. If a
   * command's requirements are not available, it will only be started if all the commands currently
   * using those requirements have been scheduled as interruptible. If this is the case, they will
   * be interrupted and the command will be scheduled.
   *
   * @param command the command to schedule. If null, no-op.
   */
  private void schedule(Command command) {
    if (command == null) {
      //      DriverStation.reportWarning("Tried to schedule a null command", true);
      return;
    }
    if (m_inRunLoop) {
      m_toSchedule.add(command);
      return;
    }

    requireNotComposed(command);

    // Do nothing if the scheduler is disabled, or the command is already scheduled.
    if (m_disabled || isScheduled(command)) {
      return;
    }

    Set<Subsystem> requirements = command.getRequirements();

    // Schedule the command if the requirements are not currently in-use.
    if (Collections.disjoint(m_requirements.keySet(), requirements)) {
      initCommand(command, requirements);
    } else {
      // Else check if the requirements that are in use have all have interruptible commands,
      // and if so, interrupt those commands and schedule the new command.
      for (Subsystem requirement : requirements) {
        Command requiring = requiring(requirement);
        if (requiring != null
            && requiring.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming) {
          return;
        }
      }
      for (Subsystem requirement : requirements) {
        Command requiring = requiring(requirement);
        if (requiring != null) {
          cancel(requiring, Optional.of(command));
        }
      }
      initCommand(command, requirements);
    }
  }

  /**
   * Schedules multiple commands for execution. Does nothing for commands already scheduled.
   *
   * @param commands the commands to schedule. No-op on null.
   */
  public void schedule(Command... commands) {
    for (Command command : commands) {
      schedule(command);
    }
  }

  /**
   * Runs a single iteration of the scheduler. The execution occurs in the following order:
   *
   * <p>Subsystem periodic methods are called.
   *
   * <p>Button bindings are polled, and new commands are scheduled from them.
   *
   * <p>Currently-scheduled commands are executed.
   *
   * <p>End conditions are checked on currently-scheduled commands, and commands that are finished
   * have their end methods called and are removed.
   *
   * <p>Any subsystems not being used as requirements have their default methods started.
   */
  public void run() {
    if (m_disabled) {
      return;
    }

    // Run the periodic method of all registered subsystems.
    for (Subsystem subsystem : m_subsystems.keySet()) {
      subsystem.periodic();
    }

    // Cache the active instance to avoid concurrency problems if setActiveLoop() is called from
    // inside the button bindings.
    EventLoop loopCache = m_activeButtonLoop;
    // Poll buttons for new commands to add.
    loopCache.poll();

    m_inRunLoop = true;
    // Run scheduled commands, remove finished commands.
    for (Iterator<Command> iterator = m_scheduledCommands.iterator(); iterator.hasNext(); ) {
      Command command = iterator.next();

      command.execute();
      for (Consumer<Command> action : m_executeActions) {
        action.accept(command);
      }
      if (command.isFinished()) {
        m_endingCommands.add(command);
        command.end(false);
        for (Consumer<Command> action : m_finishActions) {
          action.accept(command);
        }
        m_endingCommands.remove(command);
        iterator.remove();

        m_requirements.keySet().removeAll(command.getRequirements());
      }
    }
    m_inRunLoop = false;

    // Schedule/cancel commands from queues populated during loop
    for (Command command : m_toSchedule) {
      schedule(command);
    }

    for (int i = 0; i < m_toCancelCommands.size(); i++) {
      cancel(m_toCancelCommands.get(i), m_toCancelInterruptors.get(i));
    }

    m_toSchedule.clear();
    m_toCancelCommands.clear();
    m_toCancelInterruptors.clear();

    // Add default commands for un-required registered subsystems.
    for (Map.Entry<Subsystem, Command> subsystemCommand : m_subsystems.entrySet()) {
      if (!m_requirements.containsKey(subsystemCommand.getKey())
          && subsystemCommand.getValue() != null) {
        schedule(subsystemCommand.getValue());
      }
    }
  }

  /**
   * Registers subsystems with the scheduler. This must be called for the subsystem's periodic block
   * to run when the scheduler is run, and for the subsystem's default command to be scheduled. It
   * is recommended to call this from the constructor of your subsystem implementations.
   *
   * @param subsystems the subsystem to register
   */
  public void registerSubsystem(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem == null) {
        //        DriverStation.reportWarning("Tried to register a null subsystem", true);
        continue;
      }
      if (m_subsystems.containsKey(subsystem)) {
        continue;
      }
      m_subsystems.put(subsystem, null);
    }
  }

  /**
   * Un-registers subsystems with the scheduler. The subsystem will no longer have its periodic
   * block called, and will not have its default command scheduled.
   *
   * @param subsystems the subsystem to un-register
   */
  public void unregisterSubsystem(Subsystem... subsystems) {
    m_subsystems.keySet().removeAll(new HashSet<Subsystem>(Arrays.asList(subsystems)));
  }

  /**
   * Un-registers all registered Subsystems with the scheduler. All currently registered subsystems
   * will no longer have their periodic block called, and will not have their default command
   * scheduled.
   */
  public void unregisterAllSubsystems() {
    m_subsystems.clear();
  }

  /**
   * Sets the default command for a subsystem. Registers that subsystem if it is not already
   * registered. Default commands will run whenever there is no other command currently scheduled
   * that requires the subsystem. Default commands should be written to never end (i.e. their {@link
   * Command#isFinished()} method should return false), as they would simply be re-scheduled if they
   * do. Default commands must also require their subsystem.
   *
   * @param subsystem the subsystem whose default command will be set
   * @param defaultCommand the default command to associate with the subsystem
   */
  public void setDefaultCommand(Subsystem subsystem, Command defaultCommand) {
    if (subsystem == null) {
      return;
    }
    if (defaultCommand == null) {
      return;
    }

    requireNotComposed(defaultCommand);

    if (!defaultCommand.getRequirements().contains(subsystem)) {
      throw new IllegalArgumentException("Default commands must require their subsystem!");
    }

    if (defaultCommand.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming) {
      //      DriverStation.reportWarning(
      //          "Registering a non-interruptible default command!\n"
      //              + "This will likely prevent any other commands from requiring this
      // subsystem.",
      //          true);
      // Warn, but allow -- there might be a use case for this.
    }

    m_subsystems.put(subsystem, defaultCommand);
  }

  /**
   * Removes the default command for a subsystem. The current default command will run until another
   * command is scheduled that requires the subsystem, at which point the current default command
   * will not be re-scheduled.
   *
   * @param subsystem the subsystem whose default command will be removed
   */
  public void removeDefaultCommand(Subsystem subsystem) {
    if (subsystem == null) {
      return;
    }

    m_subsystems.put(subsystem, null);
  }

  /**
   * Gets the default command associated with this subsystem. Null if this subsystem has no default
   * command associated with it.
   *
   * @param subsystem the subsystem to inquire about
   * @return the default command associated with the subsystem
   */
  public Command getDefaultCommand(Subsystem subsystem) {
    return m_subsystems.get(subsystem);
  }

  /**
   * Cancels commands. The scheduler will only call {@link Command#end(boolean)} method of the
   * canceled command with {@code true}, indicating they were canceled (as opposed to finishing
   * normally).
   *
   * <p>Commands will be canceled regardless of {@link InterruptionBehavior interruption behavior}.
   *
   * @param commands the commands to cancel
   */
  public void cancel(Command... commands) {
    for (Command command : commands) {
      cancel(command, kNoInterruptor);
    }
  }

  /**
   * Cancels a command. The scheduler will only call {@link Command#end(boolean)} method of the
   * canceled command with {@code true}, indicating they were canceled (as opposed to finishing
   * normally).
   *
   * <p>Commands will be canceled regardless of {@link InterruptionBehavior interruption behavior}.
   *
   * @param command the command to cancel
   * @param interruptor the interrupting command, if any
   */
  private void cancel(Command command, Optional<Command> interruptor) {
    if (command == null) {
      //      DriverStation.reportWarning("Tried to cancel a null command", true);
      return;
    }
    if (m_endingCommands.contains(command)) {
      return;
    }
    if (m_inRunLoop) {
      m_toCancelCommands.add(command);
      m_toCancelInterruptors.add(interruptor);
      return;
    }
    if (!isScheduled(command)) {
      return;
    }

    m_endingCommands.add(command);
    command.end(true);
    for (BiConsumer<Command, Optional<Command>> action : m_interruptActions) {
      action.accept(command, interruptor);
    }
    m_endingCommands.remove(command);
    m_scheduledCommands.remove(command);
    m_requirements.keySet().removeAll(command.getRequirements());
  }

  /** Cancels all commands that are currently scheduled. */
  public void cancelAll() {
    // Copy to array to avoid concurrent modification.
    cancel(m_scheduledCommands.toArray(new Command[0]));
  }

  /**
   * Whether the given commands are running. Note that this only works on commands that are directly
   * scheduled by the scheduler; it will not work on commands inside compositions, as the scheduler
   * does not see them.
   *
   * @param commands the command to query
   * @return whether the command is currently scheduled
   */
  public boolean isScheduled(Command... commands) {
    return m_scheduledCommands.containsAll(new HashSet<Command>(Arrays.asList(commands)));
  }

  /**
   * Returns the command currently requiring a given subsystem. Null if no command is currently
   * requiring the subsystem
   *
   * @param subsystem the subsystem to be inquired about
   * @return the command currently requiring the subsystem, or null if no command is currently
   *     scheduled
   */
  public Command requiring(Subsystem subsystem) {
    return m_requirements.get(subsystem);
  }

  /** Disables the command scheduler. */
  public void disable() {
    m_disabled = true;
  }

  /** Enables the command scheduler. */
  public void enable() {
    m_disabled = false;
  }

  /**
   * Adds an action to perform on the initialization of any command by the scheduler.
   *
   * @param action the action to perform
   */
  public void onCommandInitialize(Consumer<Command> action) {
    m_initActions.add(requireNonNullParam(action, "action", "onCommandInitialize"));
  }

  /**
   * Adds an action to perform on the execution of any command by the scheduler.
   *
   * @param action the action to perform
   */
  public void onCommandExecute(Consumer<Command> action) {
    m_executeActions.add(requireNonNullParam(action, "action", "onCommandExecute"));
  }

  /**
   * Adds an action to perform on the interruption of any command by the scheduler.
   *
   * @param action the action to perform
   */
  public void onCommandInterrupt(Consumer<Command> action) {
    requireNonNullParam(action, "action", "onCommandInterrupt");
    m_interruptActions.add((command, interruptor) -> action.accept(command));
  }

  /**
   * Adds an action to perform on the interruption of any command by the scheduler. The action
   * receives the interrupted command and an Optional containing the interrupting command, or
   * Optional.empty() if it was not canceled by a command (e.g., by {@link
   * CommandScheduler#cancel}).
   *
   * @param action the action to perform
   */
  public void onCommandInterrupt(BiConsumer<Command, Optional<Command>> action) {
    m_interruptActions.add(requireNonNullParam(action, "action", "onCommandInterrupt"));
  }

  /**
   * Adds an action to perform on the finishing of any command by the scheduler.
   *
   * @param action the action to perform
   */
  public void onCommandFinish(Consumer<Command> action) {
    m_finishActions.add(requireNonNullParam(action, "action", "onCommandFinish"));
  }

  /**
   * Register commands as composed. An exception will be thrown if these commands are scheduled
   * directly or added to a composition.
   *
   * @param commands the commands to register
   * @throws IllegalArgumentException if the given commands have already been composed, or the array
   *     of commands has duplicates.
   */
  public void registerComposedCommands(Command... commands) {
    Set<Command> commandSet;
    try {
      commandSet = new HashSet<Command>(Arrays.asList(commands));
    } catch (IllegalArgumentException e) {
      throw new IllegalArgumentException(
          "Cannot compose a command twice in the same composition! (Original exception: "
              + e
              + ")");
    }
    requireNotComposedOrScheduled(commandSet);
    Exception exception = new Exception("Originally composed at:");
    exception.fillInStackTrace();
    for (Command command : commands) {
      m_composedCommands.put(command, exception);
    }
  }

  /**
   * Clears the list of composed commands, allowing all commands to be freely used again.
   *
   * <p>WARNING: Using this haphazardly can result in unexpected/undesirable behavior. Do not use
   * this unless you fully understand what you are doing.
   */
  public void clearComposedCommands() {
    m_composedCommands.clear();
  }

  /**
   * Removes a single command from the list of composed commands, allowing it to be freely used
   * again.
   *
   * <p>WARNING: Using this haphazardly can result in unexpected/undesirable behavior. Do not use
   * this unless you fully understand what you are doing.
   *
   * @param command the command to remove from the list of grouped commands
   */
  public void removeComposedCommand(Command command) {
    m_composedCommands.remove(command);
  }

  /**
   * Requires that the specified command hasn't already been added to a composition.
   *
   * @param commands The commands to check
   * @throws IllegalArgumentException if the given commands have already been composed.
   */
  public void requireNotComposed(Command... commands) {
    for (Command command : commands) {
      Exception exception = m_composedCommands.getOrDefault(command, null);
      if (exception != null) {
        throw new IllegalArgumentException(
            "Commands that have been composed may not be added to another composition or scheduled "
                + "individually!",
            exception);
      }
    }
  }

  /**
   * Requires that the specified commands have not already been added to a composition.
   *
   * @param commands The commands to check
   * @throws IllegalArgumentException if the given commands have already been composed.
   */
  public void requireNotComposed(Collection<Command> commands) {
    requireNotComposed(commands.toArray(new Command[0]));
  }

  /**
   * Requires that the specified command hasn't already been added to a composition, and is not
   * currently scheduled.
   *
   * @param command The command to check
   * @throws IllegalArgumentException if the given command has already been composed or scheduled.
   */
  public void requireNotComposedOrScheduled(Command command) {
    if (isScheduled(command)) {
      throw new IllegalArgumentException(
          "Commands that have been scheduled individually may not be added to a composition!");
    }
    requireNotComposed(command);
  }

  /**
   * Requires that the specified commands have not already been added to a composition, and are not
   * currently scheduled.
   *
   * @param commands The commands to check
   * @throws IllegalArgumentException if the given commands have already been composed or scheduled.
   */
  public void requireNotComposedOrScheduled(Collection<Command> commands) {
    for (Command command : commands) {
      requireNotComposedOrScheduled(command);
    }
  }

  /**
   * Check if the given command has been composed.
   *
   * @param command The command to check
   * @return true if composed
   */
  public boolean isComposed(Command command) {
    return getComposedCommands().contains(command);
  }

  Set<Command> getComposedCommands() {
    return m_composedCommands.keySet();
  }
}
