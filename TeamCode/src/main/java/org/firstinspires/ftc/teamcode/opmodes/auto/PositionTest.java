// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;

@Autonomous
public class PositionTest extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this);

    enabled()
        .onTrue(
            sequence(
                drivebase.alignToPose(new Pose2d(.2, 0, Rotation2d.k180deg)),
                drivebase.alignToPose(new Pose2d(0, 0, Rotation2d.kZero))));
  }
}
