// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.path;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;

public interface Trajectory {
  TrajectorySample sample(double time);

  record TrajectorySample(Pose2d pose, ChassisSpeeds speeds) {}
}
