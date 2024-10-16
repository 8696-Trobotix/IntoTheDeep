// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.path;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;

public interface Trajectory {
  Sample sample(double time);

  record Constraints(
      double maxLinearSpeedMetersPerSec,
      double maxLinearAccelMetersPerSecSquared,
      double maxAngularSpeedRadPerSec,
      double maxAngularAccelRadPerSecSquared) {}

  record Sample(Pose2d pose, ChassisSpeeds speeds) {}

  static LinearTrajectory linear(Pose2d start, Pose2d end, Constraints constraints) {
    return new LinearTrajectory(start, end, constraints);
  }
}
