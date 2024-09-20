// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.lib.teamlib.kinematics;

import org.firstinspires.ftc.teamcode.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.WheelPositions;

/** Represents the wheel positions for a omni wheel drivetrain. */
public class OmniWheelPositions implements WheelPositions<OmniWheelPositions> {
  public double[] positions;

  public OmniWheelPositions(double... wheelPositions) {
    this.positions = wheelPositions;
  }

  public OmniWheelPositions() {
    this(0, 0, 0);
  }

  @Override
  public OmniWheelPositions copy() {
    return new OmniWheelPositions(positions);
  }

  @Override
  public OmniWheelPositions interpolate(OmniWheelPositions endValue, double t) {
    double[] newPositions = new double[positions.length];
    for (int i = 0; i < positions.length; i++) {
      newPositions[i] = MathUtil.interpolate(positions[i], endValue.positions[i], t);
    }
    return new OmniWheelPositions(newPositions);
  }

  public OmniWheelPositions plus(OmniWheelPositions wheelPositions) {
    double[] newPositions = new double[this.positions.length];
    for (int i = 0; i < this.positions.length; i++) {
      newPositions[i] = this.positions[i] + wheelPositions.positions[i];
    }
    return new OmniWheelPositions(newPositions);
  }

  public OmniWheelPositions minus(OmniWheelPositions wheelPositions) {
    return plus(wheelPositions.unaryMinus());
  }

  public OmniWheelPositions unaryMinus() {
    double[] newPositions = new double[this.positions.length];
    for (int i = 0; i < this.positions.length; i++) {
      newPositions[i] = -this.positions[i];
    }
    return new OmniWheelPositions(newPositions);
  }

  public OmniWheelPositions times(double scalar) {
    double[] newPositions = new double[this.positions.length];
    for (int i = 0; i < this.positions.length; i++) {
      newPositions[i] = this.positions[i] * scalar;
    }
    return new OmniWheelPositions(newPositions);
  }

  public OmniWheelPositions divide(double scalar) {
    return times(1.0 / scalar);
  }
}
