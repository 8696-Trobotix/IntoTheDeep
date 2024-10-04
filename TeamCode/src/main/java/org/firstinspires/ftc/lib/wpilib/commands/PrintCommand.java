// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.commands;

/**
 * A command that prints a string when initialized.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class PrintCommand extends InstantCommand {
  /**
   * Creates a new a PrintCommand.
   *
   * @param message the message to print
   */
  public PrintCommand(String message) {
    super(() -> System.out.println(message));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
