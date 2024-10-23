// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.controller;

import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.controller.SimpleArmFeedforward;

/**
 * A wrapper for {@link PIDController} and {@link SimpleArmFeedforward} to make combining the two
 * cleaner.
 */
public class SimpleArmPIDFController {
  private final PIDController pidController;
  private final SimpleArmFeedforward feedforward;

  public SimpleArmPIDFController(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    pidController = new PIDController(kP, kI, kD);
    feedforward = new SimpleArmFeedforward(kS, kG, kV);
  }

  public SimpleArmPIDFController(double kP, double kS, double kG, double kV) {
    this(kP, 0, 0, kS, kG, kV, 0);
  }

  public double calculate(double measuredVel, double measuredAngle, double setpoint) {
    return pidController.calculate(measuredVel, setpoint)
        + feedforward.calculate(measuredAngle, setpoint);
  }
}
