// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.arm.Arm;
import org.firstinspires.ftc.teamcode.hardware.arm.Claw;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivebase;

@TeleOp
public class TeleopDrive extends BaseOpMode {
  @Override
  public void startup() {
    var arm = new Arm(this);
    var claw = new Claw(this);
    var drivebase = new Drivebase(this);

    arm.setDefaultCommand(arm.maintainAngle());
    primaryController().y().whileTrue(arm.raise());
    primaryController().a().whileTrue(arm.lower());

    primaryController().x().whileTrue(claw.open()).whileFalse(claw.close());

    drivebase.setDefaultCommand(
        drivebase.teleopDrive(
            () -> -primaryController().getLeftY(),
            () -> -primaryController().getLeftX(),
            () -> -primaryController().getRightX()));
  }
}
