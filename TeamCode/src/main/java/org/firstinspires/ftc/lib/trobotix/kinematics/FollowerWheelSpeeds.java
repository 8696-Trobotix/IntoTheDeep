// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

public class FollowerWheelSpeeds {
  public double omegaRadiansPerSecond;
  public double[] wheelSpeedsMetersPerSec;

  public FollowerWheelSpeeds(double omegaRadiansPerSecond, double... wheelSpeedsMetersPerSec) {
    this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    this.wheelSpeedsMetersPerSec = wheelSpeedsMetersPerSec;
  }
}
