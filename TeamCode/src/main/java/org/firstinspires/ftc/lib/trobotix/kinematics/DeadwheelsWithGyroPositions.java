// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.WheelPositions;

public class DeadwheelsWithGyroPositions implements WheelPositions<DeadwheelsWithGyroPositions> {
  public double xMeters;
  public double yMeters;
  public Rotation2d yaw;

  public DeadwheelsWithGyroPositions() {
    this(0, 0, new Rotation2d());
  }

  public DeadwheelsWithGyroPositions(double xMeters, double yMeters, Rotation2d yaw) {
    this.xMeters = xMeters;
    this.yMeters = yMeters;
    this.yaw = yaw;
  }

  public DeadwheelsWithGyroPositions plus(DeadwheelsWithGyroPositions other) {
    return new DeadwheelsWithGyroPositions(xMeters + other.xMeters, yMeters + other.yMeters, yaw.plus(other.yaw));
  }

  public DeadwheelsWithGyroPositions minus(DeadwheelsWithGyroPositions other) {
    return plus(other.unaryMinus());
  }

  public DeadwheelsWithGyroPositions unaryMinus() {
    return new DeadwheelsWithGyroPositions(-xMeters, -yMeters, yaw.unaryMinus());
  }

  @Override
  public DeadwheelsWithGyroPositions copy() {
    return new DeadwheelsWithGyroPositions(xMeters, yMeters, yaw);
  }

  @Override
  public DeadwheelsWithGyroPositions interpolate(DeadwheelsWithGyroPositions endValue, double t) {
    return new DeadwheelsWithGyroPositions(
        MathUtil.interpolate(xMeters, endValue.xMeters, t),
        MathUtil.interpolate(yMeters, endValue.yMeters, t),
        yaw.interpolate(endValue.yaw, yMeters));
  }
}
