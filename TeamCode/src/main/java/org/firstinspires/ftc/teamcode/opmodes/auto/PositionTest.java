// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;

@Disabled
@Autonomous
public class PositionTest extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this);

    registerAutoCommand(
        sequence(
            drivebase.setSpeedMult(1),
            drivebase.alignToPose(new Pose2d(Units.feetToMeters(6), 0, Rotation2d.kZero)),
            drivebase.alignToPose(new Pose2d(0, 0, Rotation2d.kZero))));
  }
}
