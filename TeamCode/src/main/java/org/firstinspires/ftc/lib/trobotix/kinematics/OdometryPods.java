// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.Odometry;

public class OdometryPods extends Odometry<OdometryPodWheelPositions> {
  public OdometryPods(Transform2d... pods) {
    super(
        new OdometryPodKinematics(pods),
        Rotation2d.kZero,
        new OdometryPodWheelPositions(new double[pods.length]),
        Pose2d.kZero);
  }
}
