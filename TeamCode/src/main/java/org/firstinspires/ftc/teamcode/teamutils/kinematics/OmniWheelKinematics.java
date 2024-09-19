// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.teamutils.kinematics;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Twist2d;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.Kinematics;

public class OmniWheelKinematics implements Kinematics<OmniWheelSpeeds, OmniWheelPositions> {
  private final Transform2d[] wheelPositions;

  private final SimpleMatrix inverseKinematics;
  private final SimpleMatrix forwardKinematics;

  public OmniWheelKinematics(Transform2d[] wheelPositions) {
    this.wheelPositions = wheelPositions;

    // Columns:
    // 0: x
    // 1: y
    // 2: angle
    inverseKinematics = new SimpleMatrix(wheelPositions.length, 3, true);
    setInverseKinematics(wheelPositions);
    forwardKinematics = inverseKinematics.pseudoInverse();
  }

  @Override
  public ChassisSpeeds toChassisSpeeds(OmniWheelSpeeds wheelSpeeds) {
    SimpleMatrix wheelSpeedsVector =
        new SimpleMatrix(wheelSpeeds.speeds.length, 1, true, wheelSpeeds.speeds);
    SimpleMatrix chassisSpeedsVector = forwardKinematics.mult(wheelSpeedsVector);
    return new ChassisSpeeds(
        chassisSpeedsVector.get(0, 0),
        chassisSpeedsVector.get(1, 0),
        chassisSpeedsVector.get(2, 0));
  }

  @Override
  public OmniWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return toWheelSpeeds(chassisSpeeds, new Translation2d());
  }

  private Translation2d m_prevCoR = new Translation2d();

  /**
   * Performs inverse kinematics to return the wheel speeds from a desired chassis velocity. This
   * method is often used to convert joystick values into wheel speeds.
   *
   * <p>This function also supports variable centers of rotation. During normal operations, the
   * center of rotation is usually the same as the physical center of the robot; therefore, the
   * argument is defaulted to that use case. However, if you wish to change the center of rotation
   * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
   *     component, the robot will rotate around that corner.
   * @return The wheel speeds. Use caution because they are not normalized. Sometimes, a user input
   *     may cause one of the wheel speeds to go above the attainable max velocity. Use the {@link
   *     OmniWheelSpeeds#desaturateWheelSpeeds(OmniWheelSpeeds, double)} function to rectify this
   *     issue.
   */
  public OmniWheelSpeeds toWheelSpeeds(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
    // We have a new center of rotation. We need to compute the matrix again.
    if (!centerOfRotationMeters.equals(m_prevCoR)) {
      Transform2d[] newPositions = new Transform2d[wheelPositions.length];
      for (int i = 0; i < newPositions.length; i++) {
        newPositions[i] =
            wheelPositions[i].plus(
                new Transform2d(centerOfRotationMeters.unaryMinus(), new Rotation2d()));
      }
      setInverseKinematics(newPositions);

      m_prevCoR = centerOfRotationMeters;
    }

    SimpleMatrix chassisSpeedsVector =
        new SimpleMatrix(
            3,
            1,
            true,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);
    SimpleMatrix wheelsVector = inverseKinematics.mult(chassisSpeedsVector);

    double[] retSpeeds = new double[wheelPositions.length];
    for (int i = 0; i < retSpeeds.length; i++) {
      retSpeeds[i] = wheelsVector.get(i, 0);
    }
    return new OmniWheelSpeeds(retSpeeds);
  }

  @Override
  public Twist2d toTwist2d(OmniWheelPositions start, OmniWheelPositions end) {
    return toTwist2d(end.minus(start));
  }

  public Twist2d toTwist2d(OmniWheelPositions positionDeltas) {
    SimpleMatrix moduleDeltaMatrix =
        new SimpleMatrix(positionDeltas.positions.length, 1, true, positionDeltas.positions);
    SimpleMatrix chassisDeltaVector = forwardKinematics.mult(moduleDeltaMatrix);

    return new Twist2d(
        chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
  }

  private void setInverseKinematics(Transform2d[] newWheelPositions) {
    for (int i = 0; i < newWheelPositions.length; i++) {
      inverseKinematics.setRow(
          i,
          0,
          newWheelPositions[i].getRotation().getCos(),
          newWheelPositions[i].getRotation().getSin(),
          newWheelPositions[i]
                  .getRotation()
                  .minus(newWheelPositions[i].getTranslation().getAngle())
                  .getSin()
              / newWheelPositions[i].getTranslation().getNorm());
    }
  }
}
