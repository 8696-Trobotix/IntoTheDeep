// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.commands;

import static org.firstinspires.ftc.lib.wpilib.util.ErrorMessages.requireNonNullParam;

import java.util.Map;
import java.util.function.Supplier;

/**
 * A command composition that runs one of a selection of commands using a selector and a key to
 * command mapping.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 *
 * @param <K> The type of key used to select the command
 */
public class SelectCommand<K> extends Command {
  private final Map<K, Command> m_commands;
  private final Supplier<? extends K> m_selector;
  private Command m_selectedCommand;
  private boolean m_runsWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  private final Command m_defaultCommand =
      new PrintCommand("SelectCommand selector value does not correspond to any command!");

  /**
   * Creates a new SelectCommand.
   *
   * @param commands the map of commands to choose from
   * @param selector the selector to determine which command to run
   */
  public SelectCommand(Map<K, Command> commands, Supplier<? extends K> selector) {
    m_commands = requireNonNullParam(commands, "commands", "SelectCommand");
    m_selector = requireNonNullParam(selector, "selector", "SelectCommand");

    CommandScheduler.getInstance().registerComposedCommands(m_defaultCommand);
    CommandScheduler.getInstance()
        .registerComposedCommands(commands.values().toArray(new Command[] {}));

    for (Command command : m_commands.values()) {
      m_requirements.addAll(command.getRequirements());
      m_runsWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public void initialize() {
    m_selectedCommand = m_commands.getOrDefault(m_selector.get(), m_defaultCommand);
    m_selectedCommand.initialize();
  }

  @Override
  public void execute() {
    m_selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runsWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }
}
