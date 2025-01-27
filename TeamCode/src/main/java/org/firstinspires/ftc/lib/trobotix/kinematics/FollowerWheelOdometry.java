// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;

public class FollowerWheelOdometry {
  private FollowerWheelKinematics kinematics;
  private Pose2d poseMeters;

  private FollowerWheelPositions previousWheelPositions;

  public FollowerWheelOdometry(
      FollowerWheelKinematics kinematics,
      FollowerWheelPositions wheelPositions,
      Pose2d initialPoseMeters) {
    this.kinematics = kinematics;
    previousWheelPositions = wheelPositions;
    poseMeters = initialPoseMeters;
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param positions The current readings of the encoders and the gyro.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(FollowerWheelPositions positions, Pose2d poseMeters) {
    previousWheelPositions = positions;
    this.poseMeters = poseMeters;
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters() {
    return poseMeters;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method takes in an angle parameter which is used instead of the angular rate
   * that is calculated from forward kinematics, in addition to the current distance measurement at
   * each wheel.
   *
   * @param positions The current readings of the encoders and the gyro.
   * @return The new pose of the robot.
   */
  public Pose2d update(FollowerWheelPositions positions) {
    var twist = kinematics.toTwist2d(positions.minus(previousWheelPositions));
    var newPose = poseMeters.exp(twist);

    previousWheelPositions = previousWheelPositions.copy();
    poseMeters = newPose;

    return poseMeters;
  }
}
