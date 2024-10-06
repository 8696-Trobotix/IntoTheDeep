// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.path;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.trajectory.TrapezoidProfile;

public class LinearTrajectory implements Trajectory {
  private final Pose2d start;
  private final Pose2d end;
  private final double lineAngleRad;
  private final double lineLengthMeters;

  private final TrapezoidProfile linearProfile;
  private final TrapezoidProfile.State linearStartState = new TrapezoidProfile.State();
  private final TrapezoidProfile.State linearEndState;

  private final TrapezoidProfile angularProfile;
  private final TrapezoidProfile.State angularStartState;
  private final TrapezoidProfile.State angularEndState;

  public LinearTrajectory(
      Pose2d start,
      Pose2d end,
      double maxLinearSpeedMetersPerSec,
      double maxLinearAccelMetersPerSecSquared,
      double maxAngularSpeedRadPerSec,
      double maxAngularAccelRadPerSecSquared) {
    this.start = start;
    this.end = end;
    lineAngleRad = end.minus(start).getTranslation().getAngle().getRadians();
    lineLengthMeters = end.minus(start).getTranslation().getNorm();

    linearProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxLinearSpeedMetersPerSec, maxLinearAccelMetersPerSecSquared));
    linearEndState =
        new TrapezoidProfile.State(end.getTranslation().getDistance(start.getTranslation()), 0);

    angularProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxAngularSpeedRadPerSec, maxAngularAccelRadPerSecSquared));
    angularStartState = new TrapezoidProfile.State(start.getRotation().getRadians(), 0);
    angularEndState = new TrapezoidProfile.State(end.getRotation().getRadians(), 0);
  }

  @Override
  public TrajectorySample sample(double time) {
    var linearState = linearProfile.calculate(time, linearStartState, linearEndState);
    var angularState = angularProfile.calculate(time, angularStartState, angularEndState);

    var pose =
        new Pose2d(
            start
                .getTranslation()
                .interpolate(end.getTranslation(), linearState.position / lineLengthMeters),
            new Rotation2d(angularState.position));
    var speed =
        new ChassisSpeeds(
            linearState.velocity * Math.cos(lineAngleRad),
            linearState.velocity * Math.sin(lineAngleRad),
            angularState.velocity);

    return new TrajectorySample(pose, speed);
  }
}
