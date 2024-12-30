// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp
public class TeleopDrive extends BaseOpMode {
  @Override
  public void startup() {
    var drivebase = new Drivebase(this);
    var slide = new Slide(this);
    var claw = new Claw(this);

    drivebase.setDefaultCommand(
        drivebase.teleopDrive(
            () -> -primaryController().getLeftY(),
            () -> -primaryController().getLeftX(),
            () -> -primaryController().getRightX()));
    primaryController().b().onTrue(drivebase.enableTurbo()).onFalse(drivebase.disableTurbo());

    slide.setDefaultCommand(slide.teleopControl(() -> -secondaryController().getLeftY()));
    secondaryController().y().onTrue(slide.alignHighSpecimen());

    secondaryController().a().onTrue(claw.open());
    secondaryController().b().onTrue(claw.close());
  }
}
