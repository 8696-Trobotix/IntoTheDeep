// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;
import static org.firstinspires.ftc.lib.wpilib.commands.Commands.waitSeconds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@Autonomous
public class ScoreSpecimenThenPark extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this);
    var slide = new Slide(this);
    var claw = new Claw(this);

    //    enabled()
    //        .onTrue(
    //            sequence(
    //                drivebase.alignToPose(new Pose2d(.72, 0, Rotation2d.kZero)),
    //                drivebase.alignToPose(new Pose2d(.675, 0, Rotation2d.kZero)),
    //                drivebase.alignToPose(new Pose2d(.1, -1.2, Rotation2d.kZero))));
    enabled()
        .onTrue(
            sequence(
                claw.close(),
                slide.alignHighSpecimen(),
                slide.scoreHighSpecimen(),
                waitSeconds(.5),
                claw.open(),
                waitSeconds(.5),
                slide.retract()));
  }
}
