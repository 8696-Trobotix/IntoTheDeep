// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.parallel;
import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
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

    registerAutoCommand(
        sequence(
            drivebase.setSpeedMult(.4),
            claw.close(),
            parallel(
                drivebase.alignToPose(new Pose2d(.775, 0.2, Rotation2d.kZero)),
                slide.alignHighSpecimen()),
            slide.scoreHighSpecimen(),
            claw.open(),
            parallel(slide.retract(), drivebase.alignToPose(new Pose2d(0.1, -1, Rotation2d.kZero)))));
  }
}
