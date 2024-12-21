// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;

public class OdometryPods {
  private final OdometryPodKinematics kinematics;
  private Pose2d poseMeters;
  private OdometryPodWheelPositions previousWheelPositions = new OdometryPodWheelPositions();

  public OdometryPods(OdometryPodKinematics kinematics, Pose2d initialPose) {
    this.kinematics = kinematics;
    poseMeters = initialPose;
  }

  public OdometryPods(OdometryPodKinematics kinematics) {
    this(kinematics, Pose2d.kZero);
  }

  public OdometryPods(Transform2d... pods) {
    this(new OdometryPodKinematics(pods));
  }

  public Pose2d getPoseMeters() {
    return poseMeters;
  }

  public Pose2d update(OdometryPodWheelPositions wheelPositions) {
    var twist = kinematics.toTwist2d(previousWheelPositions, wheelPositions);

    var newPose = poseMeters.exp(twist);

    previousWheelPositions = wheelPositions.copy();
    poseMeters = newPose;

    return poseMeters;
  }
}
