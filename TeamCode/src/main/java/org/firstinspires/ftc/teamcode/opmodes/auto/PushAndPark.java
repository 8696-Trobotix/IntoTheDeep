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
  protected void startup() { // replaces void init, see BaseOpMode
    var drivebase = new Drivebase(this); // See Drivebase.java for constructor

    enabled() // Runs when start button is pressed, see BaseOpMode.java
        .onTrue( // switches trigger from false to true so sequence will run
            sequence( // goes to Commands.java, just runs through each line
                //      ChassisSpeeds
                drivebase
                    .driveVel(new ChassisSpeeds(.5, 0, 0))
                    .withTimeout(1), // sets up and runs wheels, see OmniWheelKinematics.java
                drivebase
                    .driveVel(new ChassisSpeeds(-.5, 0, 0))
                    .withTimeout(6))); // see Command.java, waits in second
  } // driveVel, see Drivebase.java, line 229, takes in ChassisSpeed, see ChassisSpeed.java, line 42
  // (vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond)
}
