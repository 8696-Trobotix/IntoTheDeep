// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.commands;

import static org.firstinspires.ftc.lib.wpilib.util.ErrorMessages.requireNonNullParam;

import java.util.function.Consumer;

/**
 * A command that runs a given runnable when it is initialized, and another runnable when it ends.
 * Useful for running and then stopping a motor, or extending and then retracting a solenoid. Has no
 * end condition as-is; either subclass it or use {@link Command#withTimeout(double)} or {@link
 * Command#until(java.util.function.BooleanSupplier)} to give it one.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class StartEndCommand extends FunctionalCommand {
  /**
   * Creates a new StartEndCommand. Will run the given runnables when the command starts and when it
   * ends.
   *
   * @param onInit the Runnable to run on command init
   * @param onEnd the Runnable to run on command end
   * @param requirements the subsystems required by this command
   */
  public StartEndCommand(Runnable onInit, Runnable onEnd, Subsystem... requirements) {
    super(
        onInit,
        () -> {},
        // we need to do some magic here to null-check `onEnd` before it's captured
        droppingParameter(requireNonNullParam(onEnd, "onEnd", "StartEndCommand")),
        () -> false,
        requirements);
  }

  private static Consumer<Boolean> droppingParameter(Runnable run) {
    return bool -> run.run();
  }
}
