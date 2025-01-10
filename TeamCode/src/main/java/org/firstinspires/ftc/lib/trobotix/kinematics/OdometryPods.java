// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.Odometry;

public class OdometryPods extends Odometry<OdometryPodWheelPositions> {
  public OdometryPods(OdometryPodKinematics kinematics) {
    super(
        kinematics,
        Rotation2d.kZero,
        new OdometryPodWheelPositions(new double[kinematics.getPodCount()]),
        Pose2d.kZero);
  }
}
