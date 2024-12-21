// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.WheelPositions;

public class OdometryPodWheelPositions implements WheelPositions<OdometryPodWheelPositions> {
  public double[] podPositions;

  public OdometryPodWheelPositions(double... podPositions) {
    this.podPositions = podPositions;
  }

  @Override
  public OdometryPodWheelPositions copy() {
    return new OdometryPodWheelPositions(podPositions.clone());
  }

  @Override
  public OdometryPodWheelPositions interpolate(OdometryPodWheelPositions endValue, double t) {
    if (podPositions.length != endValue.podPositions.length) {
      throw new IllegalArgumentException(
          "Mismatch in number of pods! Expected "
              + podPositions.length
              + ", got "
              + endValue.podPositions.length);
    }
    double[] retPositions = new double[podPositions.length];
    for (int i = 0; i < podPositions.length; i++) {
      retPositions[i] = MathUtil.interpolate(podPositions[i], endValue.podPositions[i], t);
    }
    return new OdometryPodWheelPositions(retPositions);
  }

  public OdometryPodWheelPositions minus(OdometryPodWheelPositions other) {
    if (podPositions.length != other.podPositions.length) {
      throw new IllegalArgumentException(
          "Mismatch in number of pods! Expected "
              + podPositions.length
              + ", got "
              + other.podPositions.length);
    }
    double[] retPositions = new double[podPositions.length];
    for (int i = 0; i < podPositions.length; i++) {
      retPositions[i] = podPositions[i] - other.podPositions[i];
    }
    return new OdometryPodWheelPositions(retPositions);
  }
}
