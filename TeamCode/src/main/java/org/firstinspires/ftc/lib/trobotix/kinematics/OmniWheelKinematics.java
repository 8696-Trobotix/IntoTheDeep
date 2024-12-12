// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.kinematics;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.Kinematics;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual
 * wheel speeds for an arbitrary number of omni wheels.
 *
 * <p>The inverse kinematics (converting from a desired chassis velocity to individual wheel speeds)
 * uses the relative locations of the wheels with respect to the center of rotation. The center of
 * rotation for inverse kinematics is also variable. This means that you can set your center of
 * rotation in a corner of the robot to perform special evasion maneuvers.
 *
 * <p>Forward kinematics (converting an array of wheel speeds into the overall chassis motion) is
 * performs the exact opposite of what inverse kinematics does. Since this is an overdetermined
 * system (more equations than variables), we use a least-squares approximation.
 *
 * <p>The inverse kinematics: [wheelSpeeds] = [wheelLocations] * [chassisSpeeds] We take the
 * Moore-Penrose pseudoinverse of [wheelLocations] and then multiply by [wheelSpeeds] to get our
 * chassis speeds.
 *
 * <p>Forward kinematics is also used for odometry -- determining the position of the robot on the
 * field using encoders and a gyro.
 *
 * <p>Note that in the special case of 4 wheels, angled 45 degrees, and positioned in a rectangle,
 * also known as X-drive, it is identical to mecanum kinematics.
 */
public class OmniWheelKinematics implements Kinematics<OmniWheelSpeeds, OmniWheelPositions> {
  private final Transform2d[] wheelPositions;

  private final SimpleMatrix inverseKinematics;
  private final SimpleMatrix forwardKinematics;

  public OmniWheelKinematics(Transform2d... wheelPositions) {
    if (wheelPositions.length == 0) {
      throw new IllegalArgumentException("wheelPositions is empty!");
    }
    this.wheelPositions = wheelPositions;

    inverseKinematics = new SimpleMatrix(3, wheelPositions.length, true);
    setInverseKinematics(wheelPositions);
    forwardKinematics = inverseKinematics.pseudoInverse();
  }

  @Override
  public ChassisSpeeds toChassisSpeeds(OmniWheelSpeeds wheelSpeeds) {
    var wheelSpeedsVector =
        new SimpleMatrix(wheelSpeeds.speeds.length, 1, true, wheelSpeeds.speeds);
    var chassisSpeedsVector = forwardKinematics.mult(wheelSpeedsVector);
    return new ChassisSpeeds(
        chassisSpeedsVector.get(0, 0),
        chassisSpeedsVector.get(0, 1),
        chassisSpeedsVector.get(0, 2));
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

    var chassisSpeedsVector =
        new SimpleMatrix(
            1,
            3,
            false,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);
    var wheelsVector = inverseKinematics.mult(chassisSpeedsVector);

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
    var moduleDeltaMatrix =
        new SimpleMatrix(positionDeltas.positions.length, 1, true, positionDeltas.positions);

    var chassisDeltaVector = forwardKinematics.mult(moduleDeltaMatrix);

    return new Twist2d(
        chassisDeltaVector.get(0, 0), chassisDeltaVector.get(0, 1), chassisDeltaVector.get(0, 1));
  }

  private void setInverseKinematics(Transform2d[] newWheelPositions) {
    for (int i = 0; i < newWheelPositions.length; i++) {
      // Rows:
      // 0: x
      // 1: y
      // 2: angle
      inverseKinematics.setColumn(
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
