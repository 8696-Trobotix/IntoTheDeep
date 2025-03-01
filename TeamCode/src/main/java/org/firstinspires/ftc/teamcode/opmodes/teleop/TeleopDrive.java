// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.IntakeClaw;
import org.firstinspires.ftc.teamcode.hardware.IntakeSlide;
import org.firstinspires.ftc.teamcode.hardware.ScoringClaw;
import org.firstinspires.ftc.teamcode.hardware.ScoringElevator;

@TeleOp
public class TeleopDrive extends BaseOpMode {
  @Override
  public void startup() {
    var drivebase = new Drivebase(this);
    var scoringSlide = new ScoringElevator(this);
    var scoringClaw = new ScoringClaw(this);
    var intakeSlide = new IntakeSlide(this);
    var intakeClaw = new IntakeClaw(this);

    drivebase.setDefaultCommand(
        drivebase.teleopDrive(
            () -> primaryController().getLeftY(),
            () -> primaryController().getLeftX(),
            () -> primaryController().getRightX()));
    primaryController()
        .rightTrigger()
        .onTrue(drivebase.setSpeedMult(1))
        .onFalse(drivebase.setSpeedMult(.5));
    primaryController()
        .a()
        .whileTrue(
            drivebase.alignHumanPlayer(
                () -> primaryController().getLeftY(), () -> primaryController().getLeftX()));
    primaryController()
        .y()
        .whileTrue(
            drivebase.alignSpecimen(
                () -> primaryController().getLeftY(), () -> primaryController().getLeftX()));

    scoringSlide.setDefaultCommand(scoringSlide.retractDefault());
    secondaryController()
        .y()
        .whileTrue(scoringSlide.alignHighSpecimenTeleop())
        .onFalse(scoringSlide.scoreHighSpecimen().withTimeout(.5).andThen(scoringClaw.open()));
    secondaryController()
        .rightTrigger()
        .onTrue(scoringSlide.enableOverride())
        .whileTrue(scoringSlide.manualControl(() -> secondaryController().getRightY()))
        .onFalse(scoringSlide.disableOverride());
    secondaryController().a().onTrue(scoringClaw.open()).onFalse(scoringClaw.close());

    intakeSlide.setDefaultCommand(intakeSlide.manualControl(secondaryController()::getLeftY));
    secondaryController().dPadUp().onTrue(intakeClaw.pivotDown());
    secondaryController().dPadDown().onTrue(intakeClaw.pivotUp());
    secondaryController().leftTrigger().onTrue(intakeClaw.open()).onFalse(intakeClaw.close());
  }
}
