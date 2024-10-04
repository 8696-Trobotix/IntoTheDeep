// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.commands;

/**
 * A Command that runs instantly; it will initialize, execute once, and end on the same iteration of
 * the scheduler. Users can either pass in a Runnable and a set of requirements, or else subclass
 * this command if desired.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class InstantCommand extends FunctionalCommand {
  /**
   * Creates a new InstantCommand that runs the given Runnable with the given requirements.
   *
   * @param toRun the Runnable to run
   * @param requirements the subsystems required by this command
   */
  public InstantCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, () -> {}, interrupted -> {}, () -> true, requirements);
  }

  /**
   * Creates a new InstantCommand with a Runnable that does nothing. Useful only as a no-arg
   * constructor to call implicitly from subclass constructors.
   */
  public InstantCommand() {
    this(() -> {});
  }
}
