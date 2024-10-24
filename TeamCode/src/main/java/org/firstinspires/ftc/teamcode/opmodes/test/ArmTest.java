// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.arm.Arm;
import org.firstinspires.ftc.teamcode.hardware.arm.Claw;

@TeleOp(group = "Test")
public class ArmTest extends BaseOpMode {
  @Override
  public void startup() {
    var arm = new Arm(this);
    var claw = new Claw(this);

    arm.setDefaultCommand(arm.maintainAngle());

    primaryController().y().whileTrue(arm.raise());
    primaryController().a().whileTrue(arm.lower());

    primaryController().x().whileTrue(claw.open()).whileFalse(claw.close());

    primaryController().b().whileTrue(sequence(arm.voltageSweep(1)));
  }
}
