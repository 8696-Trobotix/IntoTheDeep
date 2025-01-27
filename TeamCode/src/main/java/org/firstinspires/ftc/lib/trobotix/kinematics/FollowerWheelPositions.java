// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;

public class FollowerWheelPositions {
  public Rotation2d yaw;
  public double[] wheelPositionsMeters;

  public FollowerWheelPositions(Rotation2d yaw, double... wheelPositionsMeters) {
    this.yaw = yaw;
    this.wheelPositionsMeters = wheelPositionsMeters;
  }

  public FollowerWheelPositions minus(FollowerWheelPositions other) {
    if (wheelPositionsMeters.length != other.wheelPositionsMeters.length) {
      throw new IllegalArgumentException(
          "Mismatch in number of pods! Expected "
              + wheelPositionsMeters.length
              + ", got "
              + other.wheelPositionsMeters.length);
    }
    double[] newWheelPositions = new double[wheelPositionsMeters.length];
    for (int i = 0; i < wheelPositionsMeters.length; i++) {
      newWheelPositions[i] = wheelPositionsMeters[i] - other.wheelPositionsMeters[i];
    }
    return new FollowerWheelPositions(yaw.minus(other.yaw), newWheelPositions);
  }

  public FollowerWheelPositions copy() {
    return new FollowerWheelPositions(new Rotation2d(yaw.getRadians()), wheelPositionsMeters.clone());
  }
}
