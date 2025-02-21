// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;

public class FollowerWheelKinematics {
  private final SimpleMatrix translationalInverseKinematics;
  private final SimpleMatrix translationalForwardKinematics;
  private final double[] angularInverseKinematics;

  final int wheelCount;

  public FollowerWheelKinematics(Transform2d... wheelTransforms) {
    wheelCount = wheelTransforms.length;

    angularInverseKinematics = new double[wheelCount];
    translationalInverseKinematics = new SimpleMatrix(wheelCount, 2);

    for (int i = 0; i < wheelCount; i++) {
      translationalInverseKinematics.setRow(
          i,
          0,
          wheelTransforms[i].getRotation().getCos(),
          wheelTransforms[i].getRotation().getSin());
      angularInverseKinematics[i] =
          // Cross product between a vector that represents the translation component of the robot
          // to wheel transform, and a unit vector with the direction of the wheel
          wheelTransforms[i].getX() * wheelTransforms[i].getRotation().getSin()
              - wheelTransforms[i].getY() * wheelTransforms[i].getRotation().getCos();
    }
    translationalForwardKinematics = translationalInverseKinematics.pseudoInverse();
  }

  public ChassisSpeeds toChassisSpeeds(FollowerWheelSpeeds wheelSpeeds) {
    var translationalWheelSpeedsMatrix = new SimpleMatrix(wheelCount, 1);
    for (int i = 0; i < wheelCount; i++) {
      translationalWheelSpeedsMatrix.setRow(
          i,
          0,
          wheelSpeeds.wheelSpeedsMetersPerSec[i]
              - (angularInverseKinematics[i] * wheelSpeeds.omegaRadiansPerSecond));
    }
    var translationalChassisDeltaMatrix =
        translationalForwardKinematics.mult(translationalWheelSpeedsMatrix);

    return new ChassisSpeeds(
        translationalChassisDeltaMatrix.get(0, 0),
        translationalChassisDeltaMatrix.get(1, 0),
        wheelSpeeds.omegaRadiansPerSecond);
  }

  public FollowerWheelSpeeds toWheelSpeeds(ChassisSpeeds speeds) {
    var translationalWheelSpeedsMatrix =
        translationalInverseKinematics.mult(
            new SimpleMatrix(2, 1, false, speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    double[] wheelSpeeds = new double[wheelCount];
    for (int i = 0; i < wheelCount; i++) {
      wheelSpeeds[i] =
          translationalWheelSpeedsMatrix.get(i, 0)
              + (angularInverseKinematics[i] * speeds.omegaRadiansPerSecond);
    }

    return new FollowerWheelSpeeds(speeds.omegaRadiansPerSecond, wheelSpeeds);
  }

  public Twist2d toTwist2d(FollowerWheelPositions delta) {
    if (wheelCount != delta.wheelPositionsMeters.length) {
      throw new IllegalArgumentException(
          "Mismatch in number of pods! Expected "
              + wheelCount
              + ", got "
              + delta.wheelPositionsMeters.length);
    }

    // With a 2 pod setup, there's an ambiguous case where there's no meaningful way to distinguish
    // between a chassis rotation, and a translation that results in the same wheel movements.
    // Disambiguation is done by using the gyro to figure out the rotational component of the wheel
    // movements, and subtracting that from the overall movement to get just the translational
    // component.
    var translationalWheelDeltasMatrix = new SimpleMatrix(wheelCount, 1);
    for (int i = 0; i < wheelCount; i++) {
      translationalWheelDeltasMatrix.setRow(
          i,
          0,
          delta.wheelPositionsMeters[i] - (angularInverseKinematics[i] * delta.yaw.getRadians()));
    }
    var translationalChassisDeltaMatrix =
        translationalForwardKinematics.mult(translationalWheelDeltasMatrix);

    return new Twist2d(
        translationalChassisDeltaMatrix.get(0, 0),
        translationalChassisDeltaMatrix.get(1, 0),
        delta.yaw.getRadians());
  }
}
