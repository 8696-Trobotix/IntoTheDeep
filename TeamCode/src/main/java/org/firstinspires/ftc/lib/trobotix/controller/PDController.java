// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.controller;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;

/** Implements a PD control loop. */
public class PDController {
  // Factor for "proportional" control
  private double m_kp;

  // Factor for "derivative" control
  private double m_kd;

  private double m_maximumInput;

  private double m_minimumInput;

  // Do the endpoints wrap around? e.g. Absolute encoder
  private boolean m_continuous;

  // The error at the time of the most recent call to calculate()
  private double m_error;
  private double m_errorDerivative;

  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  private double m_prevError;

  // The error that is considered at setpoint.
  private double m_positionTolerance = 0.05;
  private double m_velocityTolerance = Double.POSITIVE_INFINITY;

  private double m_setpoint;
  private double m_measurement;

  private boolean m_haveMeasurement;
  private boolean m_haveSetpoint;

  private final DoubleSupplier dtSupplier;

  /**
   * Allocates a PIDController with the given constants for kp and kd.
   *
   * @param kp The proportional coefficient.
   * @param kd The derivative coefficient.
   * @param dTSupplier The period between controller updates in seconds.
   * @throws IllegalArgumentException if kp &lt; 0
   * @throws IllegalArgumentException if kd &lt; 0
   * @throws IllegalArgumentException if period &lt;= 0
   */
  public PDController(double kp, double kd, DoubleSupplier dTSupplier) {
    m_kp = kp;
    m_kd = kd;

    this.dtSupplier = dTSupplier;

    if (kp < 0.0) {
      throw new IllegalArgumentException("Kp must be a non-negative number!");
    }
    if (kd < 0.0) {
      throw new IllegalArgumentException("Kd must be a non-negative number!");
    }
  }

  /**
   * Sets the PID Controller gain parameters.
   *
   * <p>Set the proportional, integral, and differential coefficients.
   *
   * @param kp The proportional coefficient.
   * @param kd The derivative coefficient.
   */
  public void setPID(double kp, double kd) {
    m_kp = kp;
    m_kd = kd;
  }

  /**
   * Sets the Proportional coefficient of the PID controller gain.
   *
   * @param kp The proportional coefficient. Must be &gt;= 0.
   */
  public void setP(double kp) {
    m_kp = kp;
  }

  /**
   * Sets the Differential coefficient of the PID controller gain.
   *
   * @param kd The differential coefficient. Must be &gt;= 0.
   */
  public void setD(double kd) {
    m_kd = kd;
  }

  /**
   * Get the Proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP() {
    return m_kp;
  }

  /**
   * Get the Differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD() {
    return m_kd;
  }

  /**
   * Returns the position tolerance of this controller.
   *
   * @return the position tolerance of the controller.
   */
  public double getPositionTolerance() {
    return m_positionTolerance;
  }

  /**
   * Returns the velocity tolerance of this controller.
   *
   * @return the velocity tolerance of the controller.
   */
  public double getVelocityTolerance() {
    return m_velocityTolerance;
  }

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
    m_haveSetpoint = true;

    if (m_continuous) {
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      m_error = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
      m_error = m_setpoint - m_measurement;
    }
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  public double getSetpoint() {
    return m_setpoint;
  }

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atSetpoint() {
    return m_haveMeasurement
        && m_haveSetpoint
        && Math.abs(m_error) < m_positionTolerance
        && Math.abs(m_errorDerivative) < m_velocityTolerance;
  }

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  /** Disables continuous input. */
  public void disableContinuousInput() {
    m_continuous = false;
  }

  /**
   * Returns true if continuous input is enabled.
   *
   * @return True if continuous input is enabled.
   */
  public boolean isContinuousInputEnabled() {
    return m_continuous;
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    return m_error;
  }

  /**
   * Returns the velocity error.
   *
   * @return The velocity error.
   */
  public double getVelocityError() {
    return m_errorDerivative;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   * @return The next controller output.
   */
  public double calculate(double measurement, double setpoint) {
    m_setpoint = setpoint;
    m_haveSetpoint = true;
    return calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @return The next controller output.
   */
  public double calculate(double measurement) {
    m_measurement = measurement;
    m_prevError = m_error;
    m_haveMeasurement = true;

    if (m_continuous) {
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      m_error = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
      m_error = m_setpoint - m_measurement;
    }

    double dt = dtSupplier.getAsDouble();

    m_errorDerivative = (m_error - m_prevError) / dt;

    return m_kp * m_error + m_kd * m_errorDerivative;
  }

  /** Resets the previous error and the integral term. */
  public void reset() {
    m_error = 0;
    m_prevError = 0;
    m_errorDerivative = 0;
    m_haveMeasurement = false;
  }
}
