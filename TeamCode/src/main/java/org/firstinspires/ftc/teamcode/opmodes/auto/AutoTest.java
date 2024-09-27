// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Drivebase;
import org.firstinspires.ftc.teamcode.lib.teamlib.Utils;
import org.firstinspires.ftc.teamcode.lib.wpilib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.ChassisSpeeds;

@Photon
@Autonomous(preselectTeleOp = "DriveTest")
public class AutoTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    CommandScheduler scheduler = new CommandScheduler();

    Drivebase drivebase = new Drivebase(this);

    drivebase.setDefaultCommand(drivebase.driveVel(new ChassisSpeeds(1, 0, 0)));

    telemetry.addData("Status", "Initialized");

    waitForStart();
    telemetry.addData("Status", "Running");
    while (opModeIsActive()) {
      double startTime = Utils.getTimeSeconds();

      scheduler.run();

      telemetry.addData("Loop frequency", 1.0 / (Utils.getTimeSeconds() - startTime));
    }
  }
}
