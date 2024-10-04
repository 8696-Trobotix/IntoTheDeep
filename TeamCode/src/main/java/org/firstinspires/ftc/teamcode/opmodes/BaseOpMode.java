// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    while (opModeIsActive()) {
      double startTime = Utils.getTimeSeconds();

      CommandScheduler.getInstance().run();

      TelemetryPacket timingsPacket = new TelemetryPacket();
      double deltaTime = Utils.getTimeSeconds() - startTime;
      timingsPacket.put("Main Thread Timing (ms)", deltaTime * 1000);
      timingsPacket.put("Main Thread Frequency", 1.0 / deltaTime);

      Utils.Telemetry.addData(timingsPacket);
      Utils.Telemetry.send();
    }
  }

  public void startup() {}
}
