// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.hardware.Drivebase;

@Disabled
@Autonomous
public class PushAndPark extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this);

    enabled()
        .onTrue(
            sequence(
                drivebase.driveVel(new ChassisSpeeds(.5, 0, 0)).withTimeout(1),
                drivebase.driveVel(new ChassisSpeeds(-.5, 0, 0)).withTimeout(6)));
  }
}
