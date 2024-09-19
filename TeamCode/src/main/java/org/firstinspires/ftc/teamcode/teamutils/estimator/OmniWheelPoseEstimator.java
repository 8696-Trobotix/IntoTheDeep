package org.firstinspires.ftc.teamcode.teamutils.estimator;

import org.firstinspires.ftc.teamcode.teamutils.kinematics.OmniWheelKinematics;
import org.firstinspires.ftc.teamcode.teamutils.kinematics.OmniWheelOdometry;
import org.firstinspires.ftc.teamcode.teamutils.kinematics.OmniWheelPositions;
import org.firstinspires.ftc.teamcode.wpilib.math.Matrix;
import org.firstinspires.ftc.teamcode.wpilib.math.VecBuilder;
import org.firstinspires.ftc.teamcode.wpilib.math.estimator.PoseEstimator;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.MecanumDriveOdometry;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.MecanumDriveWheelPositions;
import org.firstinspires.ftc.teamcode.wpilib.math.numbers.N1;
import org.firstinspires.ftc.teamcode.wpilib.math.numbers.N3;

/**
 * This class wraps {@link MecanumDriveOdometry Mecanum Drive Odometry} to fuse latency-compensated
 * vision measurements with mecanum drive encoder distance measurements. It will correct for noisy
 * measurements and encoder drift. It is intended to be a drop-in replacement for {@link
 * MecanumDriveOdometry}.
 *
 * <p>{@link OmniWheelPoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link OmniWheelPoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave mostly like regular encoder odometry.
 */
public class OmniWheelPoseEstimator extends PoseEstimator<OmniWheelPositions> {
  /**
   * Constructs a MecanumDrivePoseEstimator with default standard deviations for the model and
   * vision measurements.
   *
   * <p>The default standard deviations of the model states are 0.01 meters for x, 0.01 meters for y,
   * and 0.01 radians for heading. The default standard deviations of the vision measurements are
   * 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param wheelPositions The distances driven by each wheel.
   * @param initialPoseMeters The starting pose estimate.
   */
  public OmniWheelPoseEstimator(
      OmniWheelKinematics kinematics,
      Rotation2d gyroAngle,
      OmniWheelPositions wheelPositions,
      Pose2d initialPoseMeters) {
    this(
        kinematics,
        gyroAngle,
        wheelPositions,
        initialPoseMeters,
        VecBuilder.fill(0.01, 0.01, 0.01),
        VecBuilder.fill(0.1, 0.1, 0.1));
  }

  /**
   * Constructs a MecanumDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param wheelPositions The distance measured by each wheel.
   * @param initialPoseMeters The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public OmniWheelPoseEstimator(
      OmniWheelKinematics kinematics,
      Rotation2d gyroAngle,
      OmniWheelPositions wheelPositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super(
        kinematics,
        new OmniWheelOdometry(kinematics, gyroAngle, wheelPositions, initialPoseMeters),
        stateStdDevs,
        visionMeasurementStdDevs);
  }
}
