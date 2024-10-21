// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.lib.trobotix.EndableThread;
import org.firstinspires.ftc.lib.trobotix.Motor;
import org.firstinspires.ftc.lib.trobotix.controller.SimpleArmPIDFController;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class ArmControlThread extends EndableThread {
  static final double maxSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(30);

  private final PIDController positionController;
  private final SimpleArmPIDFController velocityController;
  private final Motor motor;

  public ArmControlThread(OpMode opMode) {
    super("Arm Control Thread");

    motor = new Motor(opMode, "armMotor");
    motor.setInverted(true);
    motor.setConversionFactor(5281.1 / (Math.PI * 2));

    positionController = new PIDController(5, 0, .5);
    velocityController =
        new SimpleArmPIDFController(24 / maxSpeedRadPerSec, 0, 0, 12 / maxSpeedRadPerSec);
  }

  private volatile boolean positionControl = false;
  private volatile double targetPos = 0;
  private volatile double targetVel = 0;

  public void setTargetPos(double angleRad) {
    positionControl = true;
    targetPos = angleRad;
  }

  public void setTargetVel(double velRadPerSec) {
    positionControl = false;
    targetVel = velRadPerSec;
  }

  @Override
  public void loop() {
    if (positionControl) {
      targetVel = positionController.calculate(motor.getPosition(), targetPos);
    }
    motor.setVoltage(
        velocityController.calculate(motor.getVelocity(), targetVel, motor.getPosition()));
  }
}
