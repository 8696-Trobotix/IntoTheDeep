// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.waitSeconds;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;

public class Claw extends SubsystemBase {
  private final Servo left;
  private final Servo right;

  public Claw(BaseOpMode opMode) {
    left = opMode.hardwareMap.servo.get("clawServoLeft");
    right = opMode.hardwareMap.servo.get("clawServoRight");
  }

  public Command open() {
    return runOnce(
            () -> {
              left.setPosition(.2);
              right.setPosition(0.8);
            })
        .andThen(waitSeconds(.5));
  }

  public Command close() {
    return runOnce(
            () -> {
              left.setPosition(0);
              right.setPosition(1);
            })
        .andThen(waitSeconds(.5));
  }
}
