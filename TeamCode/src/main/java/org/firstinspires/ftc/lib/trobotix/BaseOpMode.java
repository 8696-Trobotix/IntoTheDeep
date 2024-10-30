// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.lib.wpilib.commands.CommandScheduler;
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
    EndableThread.startThreads();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {
      CommandScheduler.getInstance().run();
      double currentTime = Utils.getTimeSeconds();
      Telemetry.addTiming("Main", currentTime - lastTime);
      lastTime = currentTime;
    }
    EndableThread.endThreads();
  }

  abstract void startup();

  final Trigger enableTrigger() {
    return enableTrigger;
  }

  private final CommandXboxController primaryController = new CommandXboxController(this, true);
  private final CommandXboxController secondaryController = new CommandXboxController(this, false);

  final CommandXboxController primaryController() {
    return primaryController;
  }

  final CommandXboxController secondaryController() {
    return secondaryController;
  }
}
