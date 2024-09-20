// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.teamutils.kinematics;

import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.Odometry;

/**
 * Class for omni wheel odometry. Odometry allows you to track the robot's position on the field
 * over a course of a match using readings from your omni wheel encoders.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 *
 * <p>Deadwheel odometry can be used with this.
 */
public class OmniWheelOdometry extends Odometry<OmniWheelPositions> {
  /**
   * Constructs a OmniWheelOdometry object.
   *
   * @param kinematics The mecanum drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The distances driven by each wheel.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public OmniWheelOdometry(
      OmniWheelKinematics kinematics,
      Rotation2d gyroAngle,
      OmniWheelPositions wheelPositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
  }

  /**
   * Constructs a OmniWheelOdometry object with the default pose at the origin.
   *
   * @param kinematics The mecanum drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The distances driven by each wheel.
   */
  public OmniWheelOdometry(
      OmniWheelKinematics kinematics, Rotation2d gyroAngle, OmniWheelPositions wheelPositions) {
    this(kinematics, gyroAngle, wheelPositions, new Pose2d());
  }
}
