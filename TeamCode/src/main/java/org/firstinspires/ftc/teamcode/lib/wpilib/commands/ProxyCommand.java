// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.lib.wpilib.commands;

import static org.firstinspires.ftc.teamcode.lib.wpilib.util.ErrorMessages.requireNonNullParam;

import java.util.function.Supplier;

/**
 * Schedules the given command when this command is initialized, and ends when it ends. Useful for
 * forking off from CommandGroups. If this command is interrupted, it will cancel the command.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class ProxyCommand extends Command {
  private final Supplier<Command> m_supplier;
  private Command m_command;

  /**
   * Creates a new ProxyCommand that schedules the supplied command when initialized, and ends when
   * it is no longer scheduled. Useful for lazily creating commands at runtime.
   *
   * @param supplier the command supplier
   */
  public ProxyCommand(Supplier<Command> supplier) {
    m_supplier = requireNonNullParam(supplier, "supplier", "ProxyCommand");
  }

  /**
   * Creates a new ProxyCommand that schedules the given command when initialized, and ends when it
   * is no longer scheduled.
   *
   * @param command the command to run by proxy
   */
  @SuppressWarnings("this-escape")
  public ProxyCommand(Command command) {
    this(() -> command);
    setName("Proxy(" + command.getName() + ")");
  }

  @Override
  public void initialize() {
    m_command = m_supplier.get();
    m_command.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_command.cancel();
    }
    m_command = null;
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // because we're between `initialize` and `end`, `m_command` is necessarily not null
    // but if called otherwise and m_command is null,
    // it's UB, so we can do whatever we want -- like return true.
    return m_command == null || !m_command.isScheduled();
  }

  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   *
   * @return true. Otherwise, this proxy would cancel commands that do run when disabled.
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
