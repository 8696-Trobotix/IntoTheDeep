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
public class CycleSpecimen extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this, true);
    var slide = new Slide(this);
    var claw = new Claw(this);

    registerAutoCommand(
        sequence(
            drivebase.setSpeedMult(.6),
            claw.close(),
            // Score the preload
            parallel(
                drivebase.alignToPose(new Pose2d(.76, 0.1, Rotation2d.kZero)),
                slide.alignHighSpecimen()),
            slide.scoreHighSpecimen(),
            claw.open(),
            // Go push a piece into HP station and grab the second specimen
            parallel(
                slide.retract(),
                sequence(
                    drivebase.alignToPose(new Pose2d(0.6, -.85, Rotation2d.kZero)),
                    drivebase.alignToPose(new Pose2d(1.35, -.85, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(1.35, -1.06, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(.1, -1.06, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(-.01, -1.06, Rotation2d.k180deg)))),
            claw.close(),
            // Go score the first HP station specimen
            parallel(
                slide.alignHighSpecimen(),
                sequence(
                    waitSeconds(.2),
                    drivebase.alignToPose(new Pose2d(.4, 0.2, Rotation2d.kZero)),
                    drivebase.alignToPose(new Pose2d(.76, 0.2, Rotation2d.kZero)))),
            slide.scoreHighSpecimen(),
            claw.open(),
            // Go push another piece into HP station and grab the second specimen
            parallel(
                slide.retract(),
                sequence(
                    drivebase.alignToPose(new Pose2d(0.6, -.85, Rotation2d.kZero)),
                    drivebase.alignToPose(new Pose2d(1.35, -.85, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(1.35, -1.26, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(.1, -1.26, Rotation2d.k180deg)),
                    drivebase.alignToPose(new Pose2d(-.01, -1.06, Rotation2d.k180deg)))),
            claw.close(),
            parallel(
                slide.alignHighSpecimen(),
                sequence(
                    waitSeconds(.2),
                    drivebase.alignToPose(new Pose2d(.4, .3, Rotation2d.kZero)),
                    drivebase.alignToPose(new Pose2d(.76, .3, Rotation2d.kZero)))),
            slide.scoreHighSpecimen(),
            claw.open()));
  }
}
