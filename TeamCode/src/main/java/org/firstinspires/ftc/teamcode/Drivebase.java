// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.*;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
  private final PhotonDcMotor frontLeft;
  private final PhotonDcMotor frontRight;
  private final PhotonDcMotor backLeft;
  private final PhotonDcMotor backRight;

  private final MecanumDriveKinematics kinematics;
  private final MecanumDrivePoseEstimator odometry;

  private final SimplePIDFController frontLeftDriveController;
  private final SimplePIDFController frontRightDriveController;
  private final SimplePIDFController backLeftDriveController;
  private final SimplePIDFController backRightDriveController;

  public Drivebase(LinearOpMode opMode) {
    frontLeft = (PhotonDcMotor) opMode.hardwareMap.dcMotor.get("frontLeft");
    frontRight = (PhotonDcMotor) opMode.hardwareMap.dcMotor.get("frontRight");
    backLeft = (PhotonDcMotor) opMode.hardwareMap.dcMotor.get("backLeft");
    backRight = (PhotonDcMotor) opMode.hardwareMap.dcMotor.get("backRight");

    frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            1
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_LEFT_WHEEL_DIAMETER));
    frontRightDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            1
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_RIGHT_WHEEL_DIAMETER));
    backLeftDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            1
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_LEFT_WHEEL_DIAMETER));
    backRightDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            1
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

  double lastTime = -1;
  double frontLeftLastPos = 0;
  double frontRightLastPos = 0;
  double backLeftLastPos = 0;
  double backRightLastPos = 0;

  public void drive(ChassisSpeeds chassisSpeeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(topSpeedMetersPerSecond);
    double currentTime = System.nanoTime() / 1e9;
    if (lastTime == -1) {
      lastTime = currentTime - .1;
    }

    // We calculate motor velocity ourselves because REV sucks and only calculates velocity at 20 hz

    double frontLeftVel =
        getVelocity(
            frontLeftLastPos,
            lastTime,
            FRONT_LEFT_WHEEL_DIAMETER
                * Units.rotationsToRadians(
                    frontLeft.getCurrentPosition() / DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION),
            currentTime);
    frontLeft.setPower(
        frontLeftDriveController.calculate(frontLeftVel, wheelSpeeds.frontLeftMetersPerSecond));
    frontLeftLastPos = frontLeft.getCurrentPosition();

    double frontRightVel =
        getVelocity(
            frontRightLastPos,
            lastTime,
            FRONT_RIGHT_WHEEL_DIAMETER
                * Units.rotationsToRadians(
                    frontRight.getCurrentPosition() / DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION),
            currentTime);
    frontRight.setPower(
        frontRightDriveController.calculate(frontRightVel, wheelSpeeds.frontRightMetersPerSecond));
    frontRightLastPos = frontRight.getCurrentPosition();

    double backLeftVel =
        getVelocity(
            backLeftLastPos,
            lastTime,
            BACK_LEFT_WHEEL_DIAMETER
                * Units.rotationsToRadians(
                    backLeft.getCurrentPosition() / DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION),
            currentTime);
    backLeft.setPower(
        backLeftDriveController.calculate(backLeftVel, wheelSpeeds.rearLeftMetersPerSecond));
    backLeftLastPos = backLeft.getCurrentPosition();

    double backRightVel =
        getVelocity(
            backRightLastPos,
            lastTime,
            BACK_RIGHT_WHEEL_DIAMETER
                * Units.rotationsToRadians(
                    backRight.getCurrentPosition() / DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION),
            currentTime);
    backRight.setPower(
        backRightDriveController.calculate(backRightVel, wheelSpeeds.rearRightMetersPerSecond));
    backRightLastPos = backRight.getCurrentPosition();

    odometry.updateWithTime(
        currentTime,
        new Rotation2d(),
        new MecanumDriveWheelPositions(
            frontLeftLastPos, frontRightLastPos, backLeftLastPos, backRightLastPos));

    lastTime = currentTime;
  }

  public void addVisionMeasurement(
      Pose2d estimatedPose, double timestamp, double translationalStDev, double angularStDev) {
    odometry.addVisionMeasurement(
        estimatedPose,
        timestamp,
        VecBuilder.fill(translationalStDev, translationalStDev, angularStDev));
  }

  private double getVelocity(
      double lastPos, double lastTime, double currentPos, double currentTime) {
    return (currentPos - lastPos) / (currentTime - lastTime);
  }
}
