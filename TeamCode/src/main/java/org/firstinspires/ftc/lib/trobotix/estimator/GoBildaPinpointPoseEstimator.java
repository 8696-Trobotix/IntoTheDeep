// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.estimator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;
import org.firstinspires.ftc.lib.trobotix.Utils;
import org.firstinspires.ftc.lib.trobotix.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.Matrix;
import org.firstinspires.ftc.lib.wpilib.math.Nat;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;
import org.firstinspires.ftc.lib.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N1;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N3;

/**
 * This class wraps {@link GoBildaPinpointDriver} to fuse latency-compensated vision measurements
 * with encoder measurements.
 *
 * <p>{@link GoBildaPinpointPoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link GoBildaPinpointPoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it then this class will behave exactly like regular encoder odometry.
 */
public class GoBildaPinpointPoseEstimator {
  private final Matrix<N3, N1> odometryMatrix = new Matrix<>(Nat.N3(), Nat.N1());
  private final Matrix<N3, N3> visionMatrix = new Matrix<>(Nat.N3(), Nat.N3());

  private static final double kBufferDuration = 1.5;
  // Maps timestamps to odometry-only pose estimates
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferDuration);
  // Maps timestamps to vision updates
  // Always contains one entry before the oldest entry in m_odometryPoseBuffer, unless there have
  // been no vision measurements after the last reset
  private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

  private final GoBildaPinpointDriver odometry;

  private Pose2d poseEstimate = new Pose2d();

  public GoBildaPinpointPoseEstimator(OpMode opMode, String name) {
    this(opMode, name, VecBuilder.fill(.01, .01, .01), VecBuilder.fill(.1, .1, .1));
  }

  public GoBildaPinpointPoseEstimator(
      OpMode opMode,
      String name,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    odometry = opMode.hardwareMap.get(GoBildaPinpointDriver.class, name);

    for (int i = 0; i < 3; ++i) {
      odometryMatrix.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  public final void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    for (int row = 0; row < 3; ++row) {
      if (odometryMatrix.get(row, 0) == 0.0) {
        visionMatrix.set(row, row, 0.0);
      } else {
        visionMatrix.set(
            row,
            row,
            odometryMatrix.get(row, 0)
                / (odometryMatrix.get(row, 0) + Math.sqrt(odometryMatrix.get(row, 0) * r[row])));
      }
    }
  }

  /**
   * Resets the robot's pose.
   *
   * @param pose The pose to reset to.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(pose);
    poseBuffer.clear();
  }

  /**
   * Resets the robot's translation.
   *
   * @param translation The pose to translation to.
   */
  public void resetTranslation(Translation2d translation) {
    odometry.resetTranslation(translation);
    poseBuffer.clear();
  }

  /**
   * Resets the robot's rotation.
   *
   * @param rotation The rotation to reset to.
   */
  public void resetRotation(Rotation2d rotation) {
    odometry.resetHeading(rotation);
    poseBuffer.clear();
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return poseEstimate;
  }

  /**
   * Return the pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds The pose's timestamp in seconds.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
   */
  public Optional<Pose2d> sampleAt(double timestampSeconds) {
    // Step 0: If there are no odometry updates to sample, skip.
    if (poseBuffer.getInternalBuffer().isEmpty()) {
      return Optional.empty();
    }

    // Step 1: Make sure timestamp matches the sample from the odometry pose buffer. (When sampling,
    // the buffer will always use a timestamp between the first and last timestamps)
    double oldestOdometryTimestamp = poseBuffer.getInternalBuffer().firstKey();
    double newestOdometryTimestamp = poseBuffer.getInternalBuffer().lastKey();
    timestampSeconds =
        MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

    // Step 2: If there are no applicable vision updates, use the odometry-only information.
    if (visionUpdates.isEmpty() || timestampSeconds < visionUpdates.firstKey()) {
      return poseBuffer.getSample(timestampSeconds);
    }

    // Step 3: Get the latest vision update from before or at the timestamp to sample at.
    double floorTimestamp = visionUpdates.floorKey(timestampSeconds);
    var visionUpdate = visionUpdates.get(floorTimestamp);

    // Step 4: Get the pose measured by odometry at the time of the sample.
    var odometryEstimate = poseBuffer.getSample(timestampSeconds);

    // Step 5: Apply the vision compensation to the odometry pose.
    return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
  }

  /** Removes stale vision updates that won't affect sampling. */
  private void cleanUpVisionUpdates() {
    // Step 0: If there are no odometry samples, skip.
    if (poseBuffer.getInternalBuffer().isEmpty()) {
      return;
    }

    // Step 1: Find the oldest timestamp that needs a vision update.
    double oldestOdometryTimestamp = poseBuffer.getInternalBuffer().firstKey();

    // Step 2: If there are no vision updates before that timestamp, skip.
    if (visionUpdates.isEmpty() || oldestOdometryTimestamp < visionUpdates.firstKey()) {
      return;
    }

    // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
    double newestNeededVisionUpdateTimestamp = visionUpdates.floorKey(oldestOdometryTimestamp);

    // Step 4: Remove all entries strictly before the newest timestamp we need.
    visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * GoBildaPinpointPoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
    if (poseBuffer.getInternalBuffer().isEmpty()
        || poseBuffer.getInternalBuffer().lastKey() - kBufferDuration > timestampSeconds) {
      return;
    }

    // Step 1: Clean up any old entries
    cleanUpVisionUpdates();

    // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
    var odometrySample = poseBuffer.getSample(timestampSeconds);

    if (odometrySample.isEmpty()) {
      return;
    }

    // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
    // made.
    var visionSample = sampleAt(timestampSeconds);

    if (visionSample.isEmpty()) {
      return;
    }

    // Step 4: Measure the twist between the old pose estimate and the vision pose.
    var twist = visionSample.get().log(visionRobotPoseMeters);

    // Step 5: We should not trust the twist entirely, so instead we scale this twist by a Kalman
    // gain matrix representing how much we trust vision measurements compared to our current pose.
    var k_times_twist = visionMatrix.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    // Step 6: Convert back to Twist2d.
    var scaledTwist =
        new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

    // Step 7: Calculate and record the vision update.
    var visionUpdate = new VisionUpdate(visionSample.get().exp(scaledTwist), odometrySample.get());
    visionUpdates.put(timestampSeconds, visionUpdate);

    // Step 8: Remove later vision measurements. (Matches previous behavior)
    visionUpdates.tailMap(timestampSeconds, false).entrySet().clear();

    // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
    // it's guaranteed to be the latest vision update.
    poseEstimate = visionUpdate.compensate(odometry.getPose());
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * GoBildaPinpointPoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * GoBildaPinpointPoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   */
  public Pose2d update() {
    return updateWithTime(Utils.getTimeSeconds());
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d updateWithTime(double currentTimeSeconds) {
    odometry.update();
    var odometryEstimate = odometry.getPose();

    poseBuffer.addSample(currentTimeSeconds, odometryEstimate);

    if (visionUpdates.isEmpty()) {
      poseEstimate = odometryEstimate;
    } else {
      var visionUpdate = visionUpdates.get(visionUpdates.lastKey());
      poseEstimate = visionUpdate.compensate(odometryEstimate);
    }

    return getEstimatedPosition();
  }

  /**
   * Represents a vision update record. The record contains the vision-compensated pose estimate as
   * well as the corresponding odometry pose estimate.
   *
   * @param visionPose The vision-compensated pose estimate.
   * @param odometryPose The pose estimated based solely on odometry.
   */
  private record VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
    /**
     * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
     * relative to this record's odometry pose to being relative to this record's vision pose.
     *
     * @param pose The pose to compensate.
     * @return The compensated pose.
     */
    public Pose2d compensate(Pose2d pose) {
      var delta = pose.minus(this.odometryPose);
      return this.visionPose.plus(delta);
    }
  }
}
