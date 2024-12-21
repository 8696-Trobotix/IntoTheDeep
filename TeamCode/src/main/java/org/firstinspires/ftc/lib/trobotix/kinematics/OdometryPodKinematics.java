// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.Kinematics;

public class OdometryPodKinematics
    implements Kinematics<OdometryPodWheelSpeeds, OdometryPodWheelPositions> {
  private final SimpleMatrix inverseKinematics;
  private final SimpleMatrix forwardKinematics;

  private final int podCount;

  public OdometryPodKinematics(Transform2d... podTransforms) {
    podCount = podTransforms.length;

    inverseKinematics = new SimpleMatrix(podCount, 3);

    for (int i = 0; i < podCount; i++) {
      inverseKinematics.setRow(
          i,
          0,
          podTransforms[i].getRotation().getCos(),
          podTransforms[i].getRotation().getSin(),
          podTransforms[i]
                  .getRotation()
                  .minus(podTransforms[i].getTranslation().getAngle())
                  .getSin()
              * podTransforms[i].getTranslation().getNorm());
    }
    forwardKinematics = inverseKinematics.pseudoInverse();
  }

  @Override
  public ChassisSpeeds toChassisSpeeds(OdometryPodWheelSpeeds wheelSpeeds) {
    if (podCount != wheelSpeeds.podSpeeds.length) {
      throw new IllegalArgumentException(
          "Mismatch in number of pods! Expected "
              + podCount
              + ", got "
              + wheelSpeeds.podSpeeds.length);
    }
    var wheelSpeedsMatrix = new SimpleMatrix(podCount, 1, false, wheelSpeeds.podSpeeds);
    var chassisSpeedsMatrix = forwardKinematics.mult(wheelSpeedsMatrix);
    return new ChassisSpeeds(
        chassisSpeedsMatrix.get(0, 0),
        chassisSpeedsMatrix.get(1, 0),
        chassisSpeedsMatrix.get(2, 0));
  }

  @Override
  public OdometryPodWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    var chassisSpeedsMatrix =
        new SimpleMatrix(
            3,
            1,
            false,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);
    var wheelSpeedsMatrix = inverseKinematics.mult(chassisSpeedsMatrix);
    var retSpeeds = new double[podCount];
    for (int i = 0; i < podCount; i++) {
      retSpeeds[i] = wheelSpeedsMatrix.get(i, 0);
    }
    return new OdometryPodWheelSpeeds(retSpeeds);
  }

  public Twist2d toTwist2d(OdometryPodWheelPositions delta) {
    if (podCount != delta.podPositions.length) {
      throw new IllegalArgumentException(
          "Mismatch in number of pods! Expected "
              + podCount
              + ", got "
              + delta.podPositions.length);
    }
    var wheelDeltasMatrix = new SimpleMatrix(podCount, 1, false, delta.podPositions);
    var chassisDeltaMatrix = forwardKinematics.mult(wheelDeltasMatrix);
    return new Twist2d(
        chassisDeltaMatrix.get(0, 0), chassisDeltaMatrix.get(1, 0), chassisDeltaMatrix.get(2, 0));
  }

  @Override
  public Twist2d toTwist2d(OdometryPodWheelPositions start, OdometryPodWheelPositions end) {
    return toTwist2d(end.minus(start));
  }

  @Override
  public OdometryPodWheelPositions interpolate(
      OdometryPodWheelPositions startValue, OdometryPodWheelPositions endValue, double t) {
    return startValue.interpolate(endValue, t);
  }
}
