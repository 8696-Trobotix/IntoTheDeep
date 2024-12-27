// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.parallel;
import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;
import org.firstinspires.ftc.teamcode.hardware.Slide;

public class CycleSpecimen extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this);
    var slide = new Slide(this);

    enabled()
        .onTrue(
            sequence(
                drivebase.setPosition(new Pose2d(.205, 0, Rotation2d.kZero)),
                parallel(
                    drivebase.alignToPose(new Pose2d(.1, 0, Rotation2d.kZero)),
                    slide.scoreHighSpecimen()),
                parallel(
                    drivebase.alignToPose(new Pose2d(.8, -.8, Rotation2d.kZero)), slide.retract()),
                drivebase.alignToPose(new Pose2d(1.2, -.8, Rotation2d.kZero)),
                drivebase.alignToPose(new Pose2d(1.6, -1.1, Rotation2d.k180deg))));
  }
}
