// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.commands.button.Trigger;
import org.firstinspires.ftc.teamcode.hardware.arm.Arm;
import org.firstinspires.ftc.teamcode.hardware.arm.Claw;

public class ArmTest extends BaseOpMode {
  @Override
  public void startup() {
    var arm = new Arm(this);
    var claw = new Claw(this);

    arm.setDefaultCommand(arm.maintainAngle());

    new Trigger(() -> this.gamepad1.y).whileTrue(arm.raise());
    new Trigger(() -> this.gamepad1.a).whileTrue(arm.lower());

    new Trigger(() -> this.gamepad1.x).whileTrue(claw.open()).whileFalse(claw.close());
  }
}
