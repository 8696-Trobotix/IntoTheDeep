// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.drive;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.Utils;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.Subsystem;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class Drivebase implements Subsystem {
  private final MecanumDriveKinematics kinematics;
  //  private final OmniWheelPoseEstimator odometry;

  private final WheelControlThread wheelControlThread;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  public Drivebase(OpMode opMode) {
    kinematics =
        new MecanumDriveKinematics(
            WHEEL_POSITIONS[0], WHEEL_POSITIONS[1], WHEEL_POSITIONS[2], WHEEL_POSITIONS[3]);
    //    odometry =
    //        new OmniWheelPoseEstimator(
    //            new OmniWheelKinematics(new Transform2d[] {}),
    //            new Rotation2d(),
    //            new OmniWheelPositions(),
    //            new Pose2d());

    wheelControlThread = new WheelControlThread(opMode);

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    wheelControlThread.start();
  }

  @Override
  public void periodic() {
    // TODO: Implement sensors
    //    odometry.update(new Rotation2d(), new OmniWheelPositions());
  }

  public final double topTranslationalSpeedMetersPerSec =
      Utils.minimum(
              FRONT_LEFT_WHEEL_DIAMETER,
              FRONT_RIGHT_WHEEL_DIAMETER,
              BACK_LEFT_WHEEL_DIAMETER,
              BACK_RIGHT_WHEEL_DIAMETER)
          * Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM);
  public final double topAngularSpeedRadPerSec =
      topTranslationalSpeedMetersPerSec / Math.hypot(TRACK_LENGTH / 2, TRACK_WIDTH / 2);

  private void drive(ChassisSpeeds chassisSpeeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(topTranslationalSpeedMetersPerSec);

    wheelControlThread.setWheelSpeeds(wheelSpeeds);
  }

  private void fieldRelativeDrive(ChassisSpeeds chassisSpeeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, new Rotation2d()));
  }

  private void alignToPose(Pose2d pose) {
    //    pose = Utils.flipAllianceOnRed(pose);

    //    fieldRelativeDrive(
    //        new ChassisSpeeds(
    //            xController.calculate(odometry.getEstimatedPosition().getX(), pose.getX()),
    //            yController.calculate(odometry.getEstimatedPosition().getY(), pose.getY()),
    //            yawController.calculate(
    //                odometry.getEstimatedPosition().getRotation().getRadians(),
    //                pose.getRotation().getRadians())));
  }

  public Command teleopDrive(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput) {
    return run(
        () ->
            drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        xInput.getAsDouble() * topTranslationalSpeedMetersPerSec,
                        yInput.getAsDouble() * topTranslationalSpeedMetersPerSec,
                        omegaInput.getAsDouble() * topAngularSpeedRadPerSec),
                    new Rotation2d())));
  }

  public Command driveVel(ChassisSpeeds speeds) {
    return run(() -> drive(speeds));
  }

  public Command submersibleAlignIntake(
      DoubleSupplier strafeSupplier, DoubleSupplier intakeSupplier) {
    return run(
        () -> {
          //          double submersibleWidth = Units.inchesToMeters(30);
          //          double minY = Units.inchesToMeters(Utils.FIELD_SIZE - submersibleWidth) / 2;
          //          double maxY = Utils.FIELD_SIZE - minY;
          //
          ////          Pose2d currentPose =
          // Utils.flipAllianceOnRed(odometry.getEstimatedPosition());
          //
          //          double targetX;
          //          if (minY < currentPose.getY() && currentPose.getY() < maxY) {
          //            targetX = 34;
          //          } else {
          //            targetX = 24;
          //          }
          //          double targetY = MathUtil.interpolate(minY, maxY,
          // (strafeSupplier.getAsDouble() + 1) / 2);
          //
          //          alignToPose(new Pose2d(targetX, targetY, new Rotation2d()));
        });
  }

  public Command basketAlign() {
    return run(
        () ->
            alignToPose(
                new Pose2d(
                    Units.inchesToMeters(24),
                    Units.inchesToMeters(24),
                    Rotation2d.fromDegrees(-135))));
  }

  private void addVisionMeasurement(
      Pose2d estimatedPose, double timestamp, double translationalStDev, double angularStDev) {
    //    odometry.addVisionMeasurement(
    //        estimatedPose,
    //        timestamp,
    //        VecBuilder.fill(translationalStDev, translationalStDev, angularStDev));
  }
}
