// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import android.annotation.SuppressLint;
import androidx.annotation.NonNull;
import java.util.Objects;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;

/** Represents the wheel positions for a mecanum drive drivetrain. */
public class MecanumDriveWheelPositions implements WheelPositions<MecanumDriveWheelPositions> {
  /** Distance measured by the front left wheel. */
  public double frontLeftMeters;

  /** Distance measured by the front right wheel. */
  public double frontRightMeters;

  /** Distance measured by the rear left wheel. */
  public double rearLeftMeters;

  /** Distance measured by the rear right wheel. */
  public double rearRightMeters;

  /** Constructs a MecanumDriveWheelPositions with zeros for all member fields. */
  public MecanumDriveWheelPositions() {}

  /**
   * Constructs a MecanumDriveWheelPositions.
   *
   * @param frontLeftMeters Distance measured by the front left wheel.
   * @param frontRightMeters Distance measured by the front right wheel.
   * @param rearLeftMeters Distance measured by the rear left wheel.
   * @param rearRightMeters Distance measured by the rear right wheel.
   */
  public MecanumDriveWheelPositions(
      double frontLeftMeters,
      double frontRightMeters,
      double rearLeftMeters,
      double rearRightMeters) {
    this.frontLeftMeters = frontLeftMeters;
    this.frontRightMeters = frontRightMeters;
    this.rearLeftMeters = rearLeftMeters;
    this.rearRightMeters = rearRightMeters;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof MecanumDriveWheelPositions) {
      MecanumDriveWheelPositions other = (MecanumDriveWheelPositions) obj;
      return Math.abs(other.frontLeftMeters - frontLeftMeters) < 1E-9
          && Math.abs(other.frontRightMeters - frontRightMeters) < 1E-9
          && Math.abs(other.rearLeftMeters - rearLeftMeters) < 1E-9
          && Math.abs(other.rearRightMeters - rearRightMeters) < 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(frontLeftMeters, frontRightMeters, rearLeftMeters, rearRightMeters);
  }

  @SuppressLint("DefaultLocale")
  @NonNull
  @Override
  public String toString() {
    return String.format(
        "MecanumDriveWheelPositions(Front Left: %.2f m, Front Right: %.2f m, "
            + "Rear Left: %.2f m, Rear Right: %.2f m)",
        frontLeftMeters, frontRightMeters, rearLeftMeters, rearRightMeters);
  }

  @Override
  public MecanumDriveWheelPositions copy() {
    return new MecanumDriveWheelPositions(
        frontLeftMeters, frontRightMeters, rearLeftMeters, rearRightMeters);
  }

  @Override
  public MecanumDriveWheelPositions interpolate(MecanumDriveWheelPositions endValue, double t) {
    return new MecanumDriveWheelPositions(
        MathUtil.interpolate(this.frontLeftMeters, endValue.frontLeftMeters, t),
        MathUtil.interpolate(this.frontRightMeters, endValue.frontRightMeters, t),
        MathUtil.interpolate(this.rearLeftMeters, endValue.rearLeftMeters, t),
        MathUtil.interpolate(this.rearRightMeters, endValue.rearRightMeters, t));
  }
}
