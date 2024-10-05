// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivebase;

@TeleOp
public class DriveTest extends BaseOpMode {
  @Override
  public void startup() {
    Drivebase drivebase = new Drivebase(this);
    drivebase.setDefaultCommand(
        drivebase.teleopDrive(
            () -> -this.gamepad1.left_stick_y,
            () -> -this.gamepad1.left_stick_x,
            () -> -this.gamepad1.right_stick_x));
  }
}
