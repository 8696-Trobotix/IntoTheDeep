// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import android.annotation.SuppressLint;
import androidx.annotation.NonNull;

/** Represents the wheel speeds for a mecanum drive drivetrain. */
public class MecanumDriveWheelSpeeds {
  /** Speed of the front left wheel. */
  public double frontLeftMetersPerSecond;

  /** Speed of the front right wheel. */
  public double frontRightMetersPerSecond;

  /** Speed of the rear left wheel. */
  public double rearLeftMetersPerSecond;

  /** Speed of the rear right wheel. */
  public double rearRightMetersPerSecond;

  /** Constructs a MecanumDriveWheelSpeeds with zeros for all member fields. */
  public MecanumDriveWheelSpeeds() {}

  /**
   * Constructs a MecanumDriveWheelSpeeds.
   *
   * @param frontLeftMetersPerSecond Speed of the front left wheel.
   * @param frontRightMetersPerSecond Speed of the front right wheel.
   * @param rearLeftMetersPerSecond Speed of the rear left wheel.
   * @param rearRightMetersPerSecond Speed of the rear right wheel.
   */
  public MecanumDriveWheelSpeeds(
      double frontLeftMetersPerSecond,
      double frontRightMetersPerSecond,
      double rearLeftMetersPerSecond,
      double rearRightMetersPerSecond) {
    this.frontLeftMetersPerSecond = frontLeftMetersPerSecond;
    this.frontRightMetersPerSecond = frontRightMetersPerSecond;
    this.rearLeftMetersPerSecond = rearLeftMetersPerSecond;
    this.rearRightMetersPerSecond = rearRightMetersPerSecond;
  }

  /**
   * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
   *
   * <p>Sometimes, after inverse kinematics, the requested speed from one or more wheels may be
   * above the max attainable speed for the driving motor on that wheel. To fix this issue, one can
   * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
   * absolute threshold, while maintaining the ratio of speeds between wheels.
   *
   * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a wheel can reach.
   */
  public void desaturate(double attainableMaxSpeedMetersPerSecond) {
    double realMaxSpeed =
        Math.max(Math.abs(frontLeftMetersPerSecond), Math.abs(frontRightMetersPerSecond));
    realMaxSpeed = Math.max(realMaxSpeed, Math.abs(rearLeftMetersPerSecond));
    realMaxSpeed = Math.max(realMaxSpeed, Math.abs(rearRightMetersPerSecond));

    if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
      frontLeftMetersPerSecond =
          frontLeftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      frontRightMetersPerSecond =
          frontRightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      rearLeftMetersPerSecond =
          rearLeftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      rearRightMetersPerSecond =
          rearRightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
    }
  }

  /**
   * Adds two MecanumDriveWheelSpeeds and returns the sum.
   *
   * <p>For example, MecanumDriveWheelSpeeds{1.0, 0.5, 2.0, 1.5} + MecanumDriveWheelSpeeds{2.0, 1.5,
   * 0.5, 1.0} = MecanumDriveWheelSpeeds{3.0, 2.0, 2.5, 2.5}
   *
   * @param other The MecanumDriveWheelSpeeds to add.
   * @return The sum of the MecanumDriveWheelSpeeds.
   */
  public MecanumDriveWheelSpeeds plus(MecanumDriveWheelSpeeds other) {
    return new MecanumDriveWheelSpeeds(
        frontLeftMetersPerSecond + other.frontLeftMetersPerSecond,
        frontRightMetersPerSecond + other.frontRightMetersPerSecond,
        rearLeftMetersPerSecond + other.rearLeftMetersPerSecond,
        rearRightMetersPerSecond + other.rearRightMetersPerSecond);
  }

  /**
   * Subtracts the other MecanumDriveWheelSpeeds from the current MecanumDriveWheelSpeeds and
   * returns the difference.
   *
   * <p>For example, MecanumDriveWheelSpeeds{5.0, 4.0, 6.0, 2.5} - MecanumDriveWheelSpeeds{1.0, 2.0,
   * 3.0, 0.5} = MecanumDriveWheelSpeeds{4.0, 2.0, 3.0, 2.0}
   *
   * @param other The MecanumDriveWheelSpeeds to subtract.
   * @return The difference between the two MecanumDriveWheelSpeeds.
   */
  public MecanumDriveWheelSpeeds minus(MecanumDriveWheelSpeeds other) {
    return new MecanumDriveWheelSpeeds(
        frontLeftMetersPerSecond - other.frontLeftMetersPerSecond,
        frontRightMetersPerSecond - other.frontRightMetersPerSecond,
        rearLeftMetersPerSecond - other.rearLeftMetersPerSecond,
        rearRightMetersPerSecond - other.rearRightMetersPerSecond);
  }

  /**
   * Returns the inverse of the current MecanumDriveWheelSpeeds. This is equivalent to negating all
   * components of the MecanumDriveWheelSpeeds.
   *
   * @return The inverse of the current MecanumDriveWheelSpeeds.
   */
  public MecanumDriveWheelSpeeds unaryMinus() {
    return new MecanumDriveWheelSpeeds(
        -frontLeftMetersPerSecond,
        -frontRightMetersPerSecond,
        -rearLeftMetersPerSecond,
        -rearRightMetersPerSecond);
  }

  /**
   * Multiplies the MecanumDriveWheelSpeeds by a scalar and returns the new MecanumDriveWheelSpeeds.
   *
   * <p>For example, MecanumDriveWheelSpeeds{2.0, 2.5, 3.0, 3.5} * 2 = MecanumDriveWheelSpeeds{4.0,
   * 5.0, 6.0, 7.0}
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled MecanumDriveWheelSpeeds.
   */
  public MecanumDriveWheelSpeeds times(double scalar) {
    return new MecanumDriveWheelSpeeds(
        frontLeftMetersPerSecond * scalar,
        frontRightMetersPerSecond * scalar,
        rearLeftMetersPerSecond * scalar,
        rearRightMetersPerSecond * scalar);
  }

  /**
   * Divides the MecanumDriveWheelSpeeds by a scalar and returns the new MecanumDriveWheelSpeeds.
   *
   * <p>For example, MecanumDriveWheelSpeeds{2.0, 2.5, 1.5, 1.0} / 2 = MecanumDriveWheelSpeeds{1.0,
   * 1.25, 0.75, 0.5}
   *
   * @param scalar The scalar to divide by.
   * @return The scaled MecanumDriveWheelSpeeds.
   */
  public MecanumDriveWheelSpeeds div(double scalar) {
    return new MecanumDriveWheelSpeeds(
        frontLeftMetersPerSecond / scalar,
        frontRightMetersPerSecond / scalar,
        rearLeftMetersPerSecond / scalar,
        rearRightMetersPerSecond / scalar);
  }

  @SuppressLint("DefaultLocale")
  @NonNull
  @Override
  public String toString() {
    return String.format(
        "MecanumDriveWheelSpeeds(Front Left: %.2f m/s, Front Right: %.2f m/s, "
            + "Rear Left: %.2f m/s, Rear Right: %.2f m/s)",
        frontLeftMetersPerSecond,
        frontRightMetersPerSecond,
        rearLeftMetersPerSecond,
        rearRightMetersPerSecond);
  }
}
