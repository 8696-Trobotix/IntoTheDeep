// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.commands;

import java.util.function.BooleanSupplier;

/**
 * A command that runs a Runnable continuously. Has no end condition as-is; either subclass it or
 * use {@link Command#withTimeout(double)} or {@link Command#until(BooleanSupplier)} to give it one.
 * If you only wish to execute a Runnable once, use {@link InstantCommand}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class RunCommand extends FunctionalCommand {
  /**
   * Creates a new RunCommand. The Runnable will be run continuously until the command ends. Does
   * not run when disabled.
   *
   * @param toRun the Runnable to run
   * @param requirements the subsystems to require
   */
  public RunCommand(Runnable toRun, Subsystem... requirements) {
    super(() -> {}, toRun, interrupted -> {}, () -> false, requirements);
  }
}
