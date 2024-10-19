// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.lib.trobotix.Motor;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.controller.SimpleArmFeedforward;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class Arm extends SubsystemBase {
  private final SimpleArmFeedforward feedforward;

  private final Motor motor;
  private final double maxSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(30);

  public Arm(OpMode opMode) {
    motor = new Motor(opMode, "armMotor");
    motor.setInverted(true);
    motor.setConversionFactor(5281.1 / (Math.PI * 2));

    feedforward = new SimpleArmFeedforward(0, 0, 12 / maxSpeedRadPerSec);
  }

  private void runVel(double velRadPerSec) {
    motor.setVoltage(feedforward.calculate(motor.getPosition(), velRadPerSec));
  }

  public Command maintainAngle() {
    return run(() -> runVel(0));
  }

  public Command raise() {
        return run(() -> runVel(maxSpeedRadPerSec));
  }

  public Command lower() {
        return run(() -> runVel(-maxSpeedRadPerSec));
  }
}
