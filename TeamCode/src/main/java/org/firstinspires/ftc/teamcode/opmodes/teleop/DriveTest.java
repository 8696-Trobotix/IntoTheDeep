// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Drivebase;
import org.firstinspires.ftc.teamcode.lib.teamlib.Utils;

@Photon
@TeleOp
public class DriveTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Drivebase drivebase = new Drivebase(this);

    telemetry.addData("Status", "Initialized");

    waitForStart();
    telemetry.addData("Status", "Running");
    while (opModeIsActive()) {
      double startTime = Utils.getTimeSeconds();

      drivebase.teleopDrive(
          -this.gamepad1.left_stick_y, -this.gamepad1.left_stick_x, -this.gamepad1.right_stick_x);

      drivebase.periodic();

      telemetry.addData("Loop frequency", 1.0 / (Utils.getTimeSeconds() - startTime));
    }
  }
}
