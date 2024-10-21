// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.controller;

import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.controller.SimpleMotorFeedforward;

/**
 * A wrapper for {@link PIDController} and {@link SimpleMotorFeedforward} to make combining the two
 * cleaner.
 */
public class SimplePIDFController {
  private final PIDController pidController;
  private final SimpleMotorFeedforward feedforward;

  public SimplePIDFController(double kP, double kI, double kD, double kS, double kV, double kA) {
    pidController = new PIDController(kP, kI, kD);
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  public SimplePIDFController(double kP, double kS, double kV) {
    this(kP, 0, 0, kS, kV, 0);
  }

  public SimplePIDFController(double kP, double kV) {
    this(kP, 0, kV);
  }

  public double calculate(double measured, double setpoint) {
    return pidController.calculate(measured, setpoint) + feedforward.calculate(setpoint);
  }
}
