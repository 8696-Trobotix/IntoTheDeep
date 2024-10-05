// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.Kinematics;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
 * wheel speeds for 2 odometry pods and a gyro, and the other way around. For 3-pod setups, see
 * {@link OmniWheelKinematics}
 *
 * <p>No math needs to be done for any kinematics. In the case that the wheels are positioned such
 * that they face directly inwards, such as the following:
 *
 * <pre>
 * +-----+-----+
 * |     |     |
 * |     |     |
 * |        ---+
 * |           |
 * |           |
 * +-----------+
 * </pre>
 *
 * <p>The velocity measured by the vertical wheel directly corresponds to x velocity, and the
 * velocity measured by the horizontal wheel directly corresponds to y velocity. Angular velocity is
 * measured with the gyro.
 *
 * <p>Note that this class does not support wheel configurations that are not directly aligned with
 * the axes like above. If the wheels have non-aligned angles or are offset from the center of the
 * bot, wheel motions induced by rotation will not be accounted for.
 */
public class DeadwheelsWithGyroKinematics
    implements Kinematics<ChassisSpeeds, DeadwheelsWithGyroPositions> {
  @Override
  public ChassisSpeeds toChassisSpeeds(ChassisSpeeds wheelSpeeds) {
    return wheelSpeeds;
  }

  @Override
  public ChassisSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return chassisSpeeds;
  }

  @Override
  public Twist2d toTwist2d(DeadwheelsWithGyroPositions start, DeadwheelsWithGyroPositions end) {
    return toTwist2d(end.minus(start));
  }

  public Twist2d toTwist2d(DeadwheelsWithGyroPositions deltas) {
    return new Twist2d(deltas.xMeters, deltas.yMeters, deltas.yaw.getRadians());
  }
}
