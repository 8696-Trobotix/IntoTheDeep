// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.Odometry;

public class DeadwheelsWithGyroOdometry extends Odometry<DeadwheelsWithGyroPositions> {
  /**
   * Constructs an Odometry object.
   *
   * @param kinematics The kinematics of the drivebase.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public DeadwheelsWithGyroOdometry(
      DeadwheelsWithGyroKinematics kinematics,
      Rotation2d gyroAngle,
      DeadwheelsWithGyroPositions wheelPositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
  }

  public void update(double x, double y, Rotation2d yaw) {
    super.update(yaw, new DeadwheelsWithGyroPositions(x, y, yaw));
  }
}
