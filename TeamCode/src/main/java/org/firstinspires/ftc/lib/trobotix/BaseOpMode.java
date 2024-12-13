// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.CommandScheduler;
import org.firstinspires.ftc.lib.wpilib.commands.Subsystem;
import org.firstinspires.ftc.lib.wpilib.commands.button.Trigger;

/**
 * A base op mode that contains shared code. As all code defined in an op mode is in the init stage,
 * and the active running is handled by the {@link CommandScheduler}, op mode classes only need to
 * override {@link BaseOpMode#startup()}.
 */
@Photon
public abstract class BaseOpMode extends LinearOpMode {
  private final Trigger enableTrigger = new Trigger(this::opModeIsActive);

  @Override
  public final void runOpMode() {
    startup();
    waitForStart();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {
      CommandScheduler.getInstance().run();
      double currentTime = Utils.getTimeSeconds();
      Telemetry.addTiming("Main", currentTime - lastTime);
      lastTime = currentTime;
    }
  }

  /**
   * All startup code is to be implemented in this method.
   *
   * <p>This method is for defining {@link Command}s and {@link Subsystem}s to be used by the {@link
   * CommandScheduler}.
   *
   * <p>Any code that needs to run periodically either should be implemented in a {@link Command} or
   * in {@link Subsystem#periodic()}
   */
  protected abstract void startup();

  /**
   * A {@link Trigger} that flips true when "Start" is pressed on the driver station.
   *
   * <p>This is recommended for starting autos, with {@link Trigger#onTrue(Command)}
   */
  protected final Trigger enableTrigger() {
    return enableTrigger;
  }

  private final CommandXboxController primaryController = new CommandXboxController(this, true);
  private final CommandXboxController secondaryController = new CommandXboxController(this, false);

  /**
   * A {@link CommandXboxController} that wraps gamepad1 to add extra functionality.
   *
   * <p>USERS SHOULD NOT USE GAMEPAD1 DIRECTLY, IT WILL NOT UPDATE. USE THIS INSTEAD.
   */
  protected final CommandXboxController primaryController() {
    return primaryController;
  }

  /**
   * A {@link CommandXboxController} that wraps gamepad2 to add extra functionality.
   *
   * <p>USERS SHOULD NOT USE GAMEPAD2 DIRECTLY, IT WILL NOT UPDATE. USE THIS INSTEAD.
   */
  protected final CommandXboxController secondaryController() {
    return secondaryController;
  }
}
