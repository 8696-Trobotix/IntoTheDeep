// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.controller;

/**
 * A helper class that computes feedforward outputs for a simple elevator (modeled as a motor acting
 * against the force of gravity).
 */
public class ElevatorFeedforward {
  /** The static gain, in volts. */
  private final double ks;

  /** The gravity gain, in volts. */
  private final double kg;

  /** The velocity gain, in V/(m/s). */
  private final double kv;

  /**
   * Creates a new ElevatorFeedforward with the specified gains and period.
   *
   * @param ks The static gain in volts.
   * @param kg The gravity gain in volts.
   * @param kv The velocity gain in V/(m/s).
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  public ElevatorFeedforward(double ks, double kg, double kv) {
    this.ks = ks;
    this.kg = kg;
    this.kv = kv;
    if (kv < 0.0) {
      throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
    }
  }

  /**
   * Returns the static gain in volts.
   *
   * @return The static gain in volts.
   */
  public double getKs() {
    return ks;
  }

  /**
   * Returns the gravity gain in volts.
   *
   * @return The gravity gain in volts.
   */
  public double getKg() {
    return kg;
  }

  /**
   * Returns the velocity gain in V/(m/s).
   *
   * @return The velocity gain.
   */
  public double getKv() {
    return kv;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint assuming continuous control
   * (acceleration is assumed to be zero).
   *
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double velocity) {
    return ks * Math.signum(velocity) + kg + kv * velocity;
  }
}
