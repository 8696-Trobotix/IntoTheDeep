// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.firstinspires.ftc.teamcode.teamutils;

import org.firstinspires.ftc.teamcode.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.wpilib.math.controller.SimpleMotorFeedforward;

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

  public SimplePIDFController(double kP, double kI, double kD, double kV) {
    this(kP, kI, kD, 0, kV, 0);
  }

  public double calculate(double measured, double setpoint) {
    return pidController.calculate(measured, setpoint) + feedforward.calculate(setpoint);
  }
}
