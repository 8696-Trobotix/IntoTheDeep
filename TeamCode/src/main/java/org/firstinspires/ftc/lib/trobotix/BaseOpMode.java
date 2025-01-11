// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.CommandScheduler;
import org.firstinspires.ftc.lib.wpilib.commands.Subsystem;
import org.firstinspires.ftc.lib.wpilib.commands.button.Trigger;
import org.firstinspires.ftc.lib.wpilib.math.filter.LinearFilter;

/**
 * A base op mode that contains shared code. As all code defined in an op mode is in the init stage,
 * and the active running is handled by the {@link CommandScheduler}, op mode classes only need to
 * override {@link BaseOpMode#startup()}.
 */
@Photon
public abstract class BaseOpMode extends LinearOpMode {
  private Trigger enableTrigger;
  private double dt = .01;
  private double busVoltage;

  private static final ArrayList<CachedValue> cachedValues = new ArrayList<>();

  @Override
  public final void runOpMode() {
    // Pre-user code initialization
    final var lynxModules = hardwareMap.getAll(LynxModule.class);
    for (var module : lynxModules) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    final var busVoltageSensor = hardwareMap.getAll(LynxVoltageSensor.class).iterator().next();
    busVoltage = busVoltageSensor.getVoltage();
    CommandScheduler.initScheduler();
    enableTrigger = new Trigger(this::opModeIsActive);
    primaryController = new CommandXboxController(this, true);
    secondaryController = new CommandXboxController(this, false);
    // User code init
    startup();
    waitForStart();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
    double lastTime = Utils.getTimeSeconds();
    // Main loop
    while (opModeIsActive()) {
      double currentTime = Utils.getTimeSeconds();
      for (var module : lynxModules) {
        module.clearBulkCache();
      }
      for (var cachedValue : cachedValues) {
        cachedValue.timestamp = currentTime;
        cachedValue.update();
      }
      dt = currentTime - lastTime;
      busVoltage = busVoltageSensor.getVoltage();
      CommandScheduler.getInstance().run();
      telemetry.addData("Main loop freq", 1.0 / dt);
      telemetry.update();
      lastTime = currentTime;
    }
    // Exit loop, manual cleanup because I can't trust the SDK/Java to handle object cleanup
    // correctly
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().clearComposedCommands();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().run();
    CommandScheduler.clearScheduler();
    enableTrigger = null;
    primaryController = null;
    secondaryController = null;
    autoCommand = null;
    cachedValues.clear();
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
  protected final Trigger enabled() {
    return enableTrigger;
  }

  /**
   * A {@link DoubleSupplier} that returns the delta time between the current tick and the last
   * tick.
   */
  protected DoubleSupplier dtSupplier() {
    return () -> dt;
  }

  private Command autoCommand;

  /**
   * Registers the {@link Command} that runs on auto start.
   *
   * <p>Normally this should be a {@link Trigger} but for unknown reasons that's flaky and
   * unreliable.
   */
  protected void registerAutoCommand(Command command) {
    autoCommand = command;
  }

  public DoubleSupplier busVoltageSupplier() {
    return () -> busVoltage;
  }

  private CommandXboxController primaryController;
  private CommandXboxController secondaryController;

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

  public abstract static class CachedValue {
    double timestamp;

    protected CachedValue() {
      cachedValues.add(this);
    }

    protected abstract void update();

    public static class CachedDouble extends CachedValue {
      public double latestValue;

      private final DoubleSupplier valueSupplier;

      public CachedDouble(DoubleSupplier valueSupplier) {
        this.valueSupplier = valueSupplier;
      }

      @Override
      protected void update() {
        latestValue = valueSupplier.getAsDouble();
      }
    }

    public static class CachedPosition extends CachedValue {
      private final DoubleSupplier positionSupplier;

      public CachedPosition(DoubleSupplier positionSupplier) {
        this.positionSupplier = positionSupplier;
      }

      public double latestPosition;
      public double latestVelocity;
      private double lastTimestamp = -1;

      private final LinearFilter velocityFilter = LinearFilter.movingAverage(3);

      @Override
      protected void update() {
        double lastPos = latestPosition;
        latestPosition = positionSupplier.getAsDouble();

        if (lastTimestamp == -1) {
          latestVelocity = 0;
        } else {
          latestVelocity =
              velocityFilter.calculate((latestPosition - lastPos) / (timestamp - lastTimestamp));
        }

        lastTimestamp = timestamp;
      }
    }
  }
}
