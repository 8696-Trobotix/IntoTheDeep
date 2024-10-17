// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.lib.trobotix.Motor;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.controller.SimpleArmFeedforward;

public class Arm extends SubsystemBase {
  private final SimpleArmFeedforward feedforward;

  private final Motor motor;

  public Arm(OpMode opMode) {
    motor = new Motor(opMode, "motor1");

    feedforward = new SimpleArmFeedforward(0, 0, 0);
  }

  private void runVel(double velRadPerSec) {
    motor.setVoltage(feedforward.calculate(getAngleRad(), velRadPerSec));
  }

  public Command raise() {
    //    return run(() -> runVel(1));
    return run(() -> motor.setVoltage(12));
  }

  public Command lower() {
    //    return run(() -> runVel(-1));
    return run(() -> motor.setVoltage(-12));
  }

  private double getAngleRad() {
    return Math.PI / 2;
  }
}
