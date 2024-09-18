// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.teamutils.Motor;
import org.firstinspires.ftc.teamcode.teamutils.SimplePIDFController;
import org.firstinspires.ftc.teamcode.wpilib.math.VecBuilder;
import org.firstinspires.ftc.teamcode.wpilib.math.estimator.MecanumDrivePoseEstimator;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.MecanumDriveWheelPositions;
import org.firstinspires.ftc.teamcode.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.teamcode.wpilib.math.utils.Units;

public class Drivebase {
  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final MecanumDriveKinematics kinematics;
  private final MecanumDrivePoseEstimator odometry;

  private final SimplePIDFController frontLeftDriveController;
  private final SimplePIDFController frontRightDriveController;
  private final SimplePIDFController backLeftDriveController;
  private final SimplePIDFController backRightDriveController;

  public Drivebase(LinearOpMode opMode) {
    frontLeft = new Motor(opMode, "frontLeft");
    frontRight = new Motor(opMode, "frontRight");
    backLeft = new Motor(opMode, "backLeft");
    backRight = new Motor(opMode, "backRight");

    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontRight.setInverted(false);
    backRight.setInverted(false);

    frontLeft.setIdleBrake(true);
    frontRight.setIdleBrake(true);
    backLeft.setIdleBrake(true);
    backRight.setIdleBrake(true);

    // Convert from encoder ticks to meters
    // Distance per rotation (meters) / encoder ticks per rotation
    frontLeft.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (FRONT_LEFT_WHEEL_DIAMETER * Math.PI));
    frontRight.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (FRONT_RIGHT_WHEEL_DIAMETER * Math.PI));
    backLeft.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (BACK_LEFT_WHEEL_DIAMETER * Math.PI));
    backRight.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (BACK_RIGHT_WHEEL_DIAMETER * Math.PI));

    kinematics =
        new MecanumDriveKinematics(
            WHEEL_POSITIONS[0], WHEEL_POSITIONS[1], WHEEL_POSITIONS[2], WHEEL_POSITIONS[3]);
    odometry =
        new MecanumDrivePoseEstimator(
            kinematics, new Rotation2d(), new MecanumDriveWheelPositions(), new Pose2d());

    frontLeftDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_LEFT_WHEEL_DIAMETER));
    frontRightDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_RIGHT_WHEEL_DIAMETER));
    backLeftDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_LEFT_WHEEL_DIAMETER));
    backRightDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_RIGHT_WHEEL_DIAMETER));
  }

  private final double topSpeedMetersPerSecond =
      Math.min(
              Math.min(FRONT_LEFT_WHEEL_DIAMETER, FRONT_RIGHT_WHEEL_DIAMETER),
              Math.min(BACK_LEFT_WHEEL_DIAMETER, FRONT_RIGHT_WHEEL_DIAMETER))
          * Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM);
  private final double drivebaseRadiusMeters = Math.hypot(TRACK_LENGTH / 2, TRACK_WIDTH / 2);

  public void teleopDrive(double xInput, double yInput, double omegaInput) {
    drive(
        new ChassisSpeeds(
            xInput * topSpeedMetersPerSecond,
            yInput * topSpeedMetersPerSecond,
            omegaInput * topSpeedMetersPerSecond / drivebaseRadiusMeters));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(topSpeedMetersPerSecond);

    frontLeft.setVoltage(
        frontLeftDriveController.calculate(
            frontLeft.getVelocity(), wheelSpeeds.frontLeftMetersPerSecond));
    frontRight.setVoltage(
        frontRightDriveController.calculate(
            frontRight.getVelocity(), wheelSpeeds.frontRightMetersPerSecond));
    backLeft.setVoltage(
        backLeftDriveController.calculate(
            backLeft.getVelocity(), wheelSpeeds.rearLeftMetersPerSecond));
    backRight.setVoltage(
        backRightDriveController.calculate(
            backRight.getVelocity(), wheelSpeeds.rearRightMetersPerSecond));

    odometry.update(
        new Rotation2d(),
        new MecanumDriveWheelPositions(
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()));
  }

  public void addVisionMeasurement(
      Pose2d estimatedPose, double timestamp, double translationalStDev, double angularStDev) {
    odometry.addVisionMeasurement(
        estimatedPose,
        timestamp,
        VecBuilder.fill(translationalStDev, translationalStDev, angularStDev));
  }
}
