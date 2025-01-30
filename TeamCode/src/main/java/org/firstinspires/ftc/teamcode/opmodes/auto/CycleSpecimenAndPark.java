// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.parallel;
import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;
import static org.firstinspires.ftc.lib.wpilib.commands.Commands.waitSeconds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@Autonomous
public class CycleSpecimenAndPark extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this, true);
    var slide = new Slide(this);
    var claw = new Claw(this);

    registerAutoCommand(
        sequence(
            drivebase.setSpeedMult(.45),
            claw.close(),
            // Score the preload
            parallel(
                drivebase.alignToPose(new Pose2d(.8, 0.1, Rotation2d.kZero)),
                slide.alignHighSpecimen()),
            slide.scoreHighSpecimen(),
            claw.open(),
            // Go push a piece into HP station and grab the second specimen
            parallel(
                slide.retract(),
                sequence(
                    drivebase.alignToPose(new Pose2d(0.6, -.7, Rotation2d.kZero)),
                    drivebase.alignToPose(new Pose2d(1.4, -.7, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(1.4, -1.03, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(0.05, -1.03, Rotation2d.k180deg)))),
            claw.close(),
            // Go score the first HP station specimen
            parallel(
                slide.alignHighSpecimen(),
                sequence(
                    waitSeconds(.2),
                    drivebase.alignToPose(new Pose2d(.6, 0.15, Rotation2d.kZero)),
                    drivebase.alignToPose(new Pose2d(.85, 0.15, Rotation2d.kZero)))),
            slide.scoreHighSpecimen(),
            claw.open(),
            // Go grab the second specimen
            parallel(
                slide.retract(),
                sequence(
                    drivebase.alignToPose(new Pose2d(.3, -1.065, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(0.065, -1.065, Rotation2d.k180deg)))),
            claw.close(),
            // Go score the second HP station specimen
            parallel(
                slide.alignHighSpecimen(),
                sequence(
                    waitSeconds(.2),
                    drivebase.alignToPose(new Pose2d(.6, .2, Rotation2d.kZero)),
                    drivebase.alignToPose(new Pose2d(.9, .2, Rotation2d.kZero)))),
            slide.scoreHighSpecimen(),
            claw.open(),
            parallel(
                slide.retract(),
                drivebase.alignToPose(new Pose2d(0.15, -1.47, Rotation2d.kZero)))));
  }
}
