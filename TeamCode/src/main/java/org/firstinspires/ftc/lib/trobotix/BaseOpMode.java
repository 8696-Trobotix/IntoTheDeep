// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.lib.wpilib.commands.CommandScheduler;

/**
 * A base op mode that contains shared code. As all code defined in an op mode is in the init stage,
 * and the active running is handled by the {@link CommandScheduler}, op mode classes only need to
 * override {@link BaseOpMode#startup()}.
 */
@Photon
public class BaseOpMode extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Utils.opModeActiveSupplier = () -> !isStopRequested();
    TelemetryThread.start();
    startup();
    waitForStart();
    while (opModeIsActive()) {
      double startTime = Utils.getTimeSeconds();
      CommandScheduler.getInstance().run();
      double schedulerTime = Utils.getTimeSeconds();

      TelemetryThread.addTiming("Main Thread", schedulerTime - startTime);
      startTime = Utils.getTimeSeconds();
    }
  }

  public void startup() {}
}
