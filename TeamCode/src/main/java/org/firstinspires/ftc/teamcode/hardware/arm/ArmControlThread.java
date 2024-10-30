// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.lib.trobotix.EndableThread;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.trobotix.controller.SimpleArmPIDFController;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class ArmControlThread extends EndableThread {
  static final double maxSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(30);

  private final PIDController positionController;
  private final SimpleArmPIDFController velocityController;
  private final Motor motor;

  private final double minAngleRad = Units.degreesToRadians(0);
  private final double maxAngleRad = Units.degreesToRadians(80);

  public ArmControlThread(OpMode opMode) {
    super("Arm Control Thread");

    motor = new Motor(opMode, "armMotor");
    motor.setInverted(false);
    motor.setConversionFactor(5281.1 / (Math.PI * 2));
    motor.setPosition(minAngleRad);

    positionController = new PIDController(0, 0, 0);
    velocityController =
        new SimpleArmPIDFController(24 / maxSpeedRadPerSec, 0, .5, 12 / maxSpeedRadPerSec);
  }

  private enum Mode {
    POSITION,
    VELOCITY,
    VOLTAGE
  }

  private volatile Mode mode = Mode.VELOCITY;
  private volatile double targetPos = minAngleRad;
  private volatile double targetVel = 0;
  private volatile double voltage = 0;

  public void setTargetPos(double angleRad) {
    mode = Mode.POSITION;
    targetPos = MathUtil.clamp(angleRad, minAngleRad, maxAngleRad);
  }

  public void setTargetVel(double velRadPerSec) {
    mode = Mode.VELOCITY;
    targetVel = velRadPerSec;
  }

  public void setVoltage(double volts) {
    mode = Mode.VOLTAGE;
    voltage = volts;
  }

  @Override
  public void loop() {
    double currentPos = motor.getPosition();
    Telemetry.addData("Arm/Position (deg)", Units.radiansToDegrees(currentPos));
    if (mode == Mode.POSITION) {
      targetVel = positionController.calculate(currentPos, targetPos);
    }
    if (mode != Mode.VOLTAGE) {
      voltage = velocityController.calculate(motor.getVelocity(), currentPos, targetVel);
    }
    //    if ((voltage < 0 && currentPos <= minAngleRad) || (voltage > 0 && currentPos >=
    // maxAngleRad)) {
    //      voltage = 0;
    //    }
    motor.setVoltage(voltage);
  }
}
