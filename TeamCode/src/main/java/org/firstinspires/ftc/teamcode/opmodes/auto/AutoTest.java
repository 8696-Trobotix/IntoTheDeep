// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Drivebase;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

@Photon
@Autonomous(preselectTeleOp = "DriveTest")
public class AutoTest extends BaseOpMode {
  @Override
  public void startup() {
    Drivebase drivebase = new Drivebase(this);

    drivebase.setDefaultCommand(drivebase.driveVel(new ChassisSpeeds(1, 0, 0)));
  }
}
