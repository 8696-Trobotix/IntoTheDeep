// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.IntakeClaw;
import org.firstinspires.ftc.teamcode.hardware.IntakeSlide;

@Disabled
@TeleOp
public class IntakeTest extends BaseOpMode {
  @Override
  protected void startup() {
    var slide = new IntakeSlide(this);
    var claw = new IntakeClaw(this);

    slide.setDefaultCommand(slide.manualControl(() -> primaryController().getLeftY()));
    primaryController().x().onTrue(claw.open());
    primaryController().y().onTrue(claw.close());
    primaryController().b().onTrue(claw.pivotDown());
    primaryController().a().onTrue(claw.pivotUp());
  }
}
