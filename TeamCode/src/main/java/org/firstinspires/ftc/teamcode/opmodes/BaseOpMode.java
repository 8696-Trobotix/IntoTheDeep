// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.teamlib.Utils;
import org.firstinspires.ftc.teamcode.lib.wpilib.commands.CommandScheduler;

@Photon
public class BaseOpMode extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    startup();
    waitForStart();
    double startTime = Utils.getTimeSeconds();
    double telemetryDeltaT = 0;
    while (opModeIsActive()) {
      CommandScheduler.getInstance().run();
      double schedulerRunTime = Utils.getTimeSeconds();
      double deltaT = schedulerRunTime - startTime;

      Utils.Telemetry.addTimings("Main Thread/Scheduler Run Time (ms)", deltaT * 1000);
      Utils.Telemetry.addTimings("Main Thread/Timings Send Time (ms)", telemetryDeltaT * 1000);
      Utils.Telemetry.addTimings("Main Thread/Frequency", 1.0 / (deltaT + telemetryDeltaT));
      Utils.Telemetry.send();

      startTime = Utils.getTimeSeconds();
      telemetryDeltaT = startTime - schedulerRunTime;
    }
  }

  public void startup() {}
}
