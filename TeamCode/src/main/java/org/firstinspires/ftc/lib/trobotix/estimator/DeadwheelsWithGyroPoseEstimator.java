// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.estimator;

import org.firstinspires.ftc.lib.trobotix.kinematics.DeadwheelsWithGyroKinematics;
import org.firstinspires.ftc.lib.trobotix.kinematics.DeadwheelsWithGyroOdometry;
import org.firstinspires.ftc.lib.trobotix.kinematics.DeadwheelsWithGyroPositions;
import org.firstinspires.ftc.lib.wpilib.math.Matrix;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.estimator.PoseEstimator;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N1;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N3;

public class DeadwheelsWithGyroPoseEstimator extends PoseEstimator<DeadwheelsWithGyroPositions> {
  /**
   * Constructs a new DeadwheelsWithGyroPoseEstimator.
   *
   * <p>The default standard deviations of the model states are 0.01 meters for x, 0.01 meters for
   * y, and 0.01 radians for heading. The default standard deviations of the vision measurements are
   * 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
   *
   * @param xMeters The initial position of the x-axis deadwheel.
   * @param yMeters The initial position of the y-axis deadwheel.
   * @param gyroAngle The initial angle of the gyro.
   * @param initialPos The initial pose for the pose estimator.
   */
  public DeadwheelsWithGyroPoseEstimator(
      double xMeters, double yMeters, Rotation2d gyroAngle, Pose2d initialPos) {
    this(
        xMeters,
        yMeters,
        gyroAngle,
        initialPos,
        VecBuilder.fill(0.01, 0.01, 0.01),
        VecBuilder.fill(0.1, 0.1, 0.1));
  }

  /**
   * Constructs a new DeadwheelsWithGyroPoseEstimator.
   *
   * @param xMeters The initial position of the x-axis deadwheel.
   * @param yMeters The initial position of the y-axis deadwheel.
   * @param gyroAngle The initial angle of the gyro.
   * @param initialPos The initial pose for the pose estimator.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public DeadwheelsWithGyroPoseEstimator(
      double xMeters,
      double yMeters,
      Rotation2d gyroAngle,
      Pose2d initialPos,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this(
        new DeadwheelsWithGyroKinematics(),
        xMeters,
        yMeters,
        gyroAngle,
        initialPos,
        stateStdDevs,
        visionMeasurementStdDevs);
  }

  private DeadwheelsWithGyroPoseEstimator(
      DeadwheelsWithGyroKinematics kinematics,
      double xMeters,
      double yMeters,
      Rotation2d gyroAngle,
      Pose2d initialPos,
      Matrix<N3, N1> stateStDevs,
      Matrix<N3, N1> visionStDevs) {
    super(
        kinematics,
        new DeadwheelsWithGyroOdometry(
            kinematics,
            gyroAngle,
            new DeadwheelsWithGyroPositions(xMeters, yMeters, gyroAngle),
            initialPos),
        stateStDevs,
        visionStDevs);
  }
}
