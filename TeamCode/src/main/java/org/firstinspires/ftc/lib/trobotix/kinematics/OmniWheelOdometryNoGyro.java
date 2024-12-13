// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;

public class OmniWheelOdometryNoGyro {
  private final OmniWheelKinematics kinematics;
  private Pose2d poseMeters = Pose2d.kZero;

  private OmniWheelPositions previousPositions;

  public OmniWheelOdometryNoGyro(Transform2d... wheelPositions) {
    this(new OmniWheelKinematics(wheelPositions), wheelPositions.length);
  }

  public OmniWheelOdometryNoGyro(OmniWheelKinematics kinematics, int wheelCount) {
    this.kinematics = kinematics;
    previousPositions = new OmniWheelPositions(new double[wheelCount]);
  }

  public Pose2d getPoseMeters() {
    return poseMeters;
  }

  public Pose2d update(OmniWheelPositions wheelPositions) {
    poseMeters = poseMeters.exp(kinematics.toTwist2d(previousPositions, wheelPositions));
    previousPositions = wheelPositions.copy();
    return poseMeters;
  }
}
