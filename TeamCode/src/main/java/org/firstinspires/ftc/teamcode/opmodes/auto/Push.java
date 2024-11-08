// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivebase;

@Autonomous
public class Push extends BaseOpMode {
  @Override
  protected void startup() {
    var drivebase = new Drivebase(this);

    enableTrigger().onTrue(drivebase.driveVel(new ChassisSpeeds(.5, 0, 0)).withTimeout(1));
  }
}
