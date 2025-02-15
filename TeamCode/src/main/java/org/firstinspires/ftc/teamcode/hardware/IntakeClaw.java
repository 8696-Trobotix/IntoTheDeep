// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.waitSeconds;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;

public class IntakeClaw extends SubsystemBase {
  private final Servo left;
  private final Servo right;
  private final Servo pivot;

  public IntakeClaw(BaseOpMode opMode) {
    left = opMode.hardwareMap.servo.get("intakeClawLeft");
    right = opMode.hardwareMap.servo.get("intakeClawRight");
    pivot = opMode.hardwareMap.servo.get("intakeClawPivot");
  }

  public Command open() {
    return runOnce(
            () -> {
              left.setPosition(.5);
              right.setPosition(0.5);
            })
        .andThen(waitSeconds(.2));
  }

  public Command close() {
    return runOnce(
            () -> {
              left.setPosition(0.75);
              right.setPosition(.25);
            })
        .andThen(waitSeconds(.2));
  }

  public Command pivotDown() {
    return runOnce(() -> pivot.setPosition(.87)).andThen(waitSeconds(1));
  }

  public Command pivotUp() {
    return runOnce(() -> pivot.setPosition(0.4)).andThen(waitSeconds(1));
  }
}
