// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.CommandScheduler;
import org.firstinspires.ftc.lib.wpilib.commands.Subsystem;
import org.firstinspires.ftc.lib.wpilib.commands.button.Trigger;

import java.util.function.DoubleSupplier;

/**
 * A base op mode that contains shared code. As all code defined in an op mode is in the init stage,
 * and the active running is handled by the {@link CommandScheduler}, op mode classes only need to
 * override {@link BaseOpMode#startup()}.
 */
@Photon
public abstract class BaseOpMode extends LinearOpMode {
  private final Trigger enableTrigger = new Trigger(this::opModeIsActive);

  private double dt = .01;

  private double busVoltage;

  @Override
  public final void runOpMode() {
    final var busVoltageSensor = hardwareMap.getAll(LynxVoltageSensor.class).iterator().next();
    busVoltage = busVoltageSensor.getVoltage();
    startup();
    waitForStart();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {
      double currentTime = Utils.getTimeSeconds();
      dt = currentTime - lastTime;
      busVoltage = busVoltageSensor.getVoltage();
      CommandScheduler.getInstance().run();
      Telemetry.addTiming("Main", dt);
      telemetry.update();
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

  /** A {@link Trigger} that flips true when the Start button is pressed. */
  protected final Trigger enableTrigger() {
    return enableTrigger;
  }

  /**
   * A {@link DoubleSupplier} that returns the delta time between the current tick and the last
   * tick.
   */
  protected DoubleSupplier dtSupplier() {
    return () -> dt;
  }

  public DoubleSupplier busVoltageSupplier() {
    return () -> busVoltage;
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
