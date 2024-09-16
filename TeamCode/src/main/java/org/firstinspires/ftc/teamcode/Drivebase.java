// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teamutils.SimplePIDFController;
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
    frontLeft = (PhotonDcMotor) opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
    frontRight = (PhotonDcMotor) opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
    backLeft = (PhotonDcMotor) opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
    backRight = (PhotonDcMotor) opMode.hardwareMap.get(DcMotorEx.class, "backRight");

    frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    kinematics =
        new MecanumDriveKinematics(
            Constants.Drivebase.WHEEL_POSITIONS[0],
            Constants.Drivebase.WHEEL_POSITIONS[1],
            Constants.Drivebase.WHEEL_POSITIONS[2],
            Constants.Drivebase.WHEEL_POSITIONS[3]);
    odometry = new MecanumDrivePoseEstimator(kinematics, new Rotation2d(), new MecanumDriveWheelPositions(), new Pose2d());

    frontLeftDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            Units.rotationsPerMinuteToRadiansPerSecond(Constants.Drivebase.DRIVE_MOTOR_MAX_RPM)
                * Constants.Drivebase.FRONT_LEFT_WHEEL_DIAMETER);
    frontRightDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            Units.rotationsPerMinuteToRadiansPerSecond(Constants.Drivebase.DRIVE_MOTOR_MAX_RPM)
                * Constants.Drivebase.FRONT_RIGHT_WHEEL_DIAMETER);
    backLeftDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            Units.rotationsPerMinuteToRadiansPerSecond(Constants.Drivebase.DRIVE_MOTOR_MAX_RPM)
                * Constants.Drivebase.BACK_LEFT_WHEEL_DIAMETER);
    backRightDriveController =
        new SimplePIDFController(
            0,
            0,
            0,
            Units.rotationsPerMinuteToRadiansPerSecond(Constants.Drivebase.DRIVE_MOTOR_MAX_RPM)
                * Constants.Drivebase.BACK_RIGHT_WHEEL_DIAMETER);
  }

  public void drive(
      double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    drive(new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond));
  }

  double lastTime = -1;
  double frontLeftLastPos = 0;
  double frontRightLastPos = 0;
  double backLeftLastPos = 0;
  double backRightLastPos = 0;

  public void drive(ChassisSpeeds chassisSpeeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    double currentTime = System.nanoTime() / 1e9;
    if (lastTime == -1) {
      lastTime = currentTime - .1;
    }

    double frontLeftVel =
        Constants.Drivebase.FRONT_LEFT_WHEEL_DIAMETER
            * Units.rotationsToRadians(
                getVelocity(frontLeftLastPos, lastTime, frontLeft.getCurrentPosition(), currentTime)
                    / Constants.Drivebase.DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION);
    frontLeft.setPower(
        frontLeftDriveController.calculate(frontLeftVel, wheelSpeeds.frontLeftMetersPerSecond));

    double frontRightVel =
        Constants.Drivebase.FRONT_RIGHT_WHEEL_DIAMETER
            * Units.rotationsToRadians(
                frontRight.getVelocity()
                    / Constants.Drivebase.DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION);
    frontRight.setPower(
        frontRightDriveController.calculate(frontRightVel, wheelSpeeds.frontRightMetersPerSecond));

    double backLeftVel =
        Constants.Drivebase.BACK_LEFT_WHEEL_DIAMETER
            * Units.rotationsToRadians(
                backLeft.getVelocity()
                    / Constants.Drivebase.DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION);
    backLeft.setPower(
        backLeftDriveController.calculate(backLeftVel, wheelSpeeds.rearLeftMetersPerSecond));

    double backRightVel =
        Constants.Drivebase.BACK_RIGHT_WHEEL_DIAMETER
            * Units.rotationsToRadians(
                backRight.getVelocity()
                    / Constants.Drivebase.DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION);
    backRight.setPower(
        backRightDriveController.calculate(backRightVel, wheelSpeeds.rearRightMetersPerSecond));

    lastTime = currentTime;
  }

  private double getVelocity(
      double lastPos, double lastTime, double currentPos, double currentTime) {
    return (currentPos - lastPos) / (currentTime - lastTime);
  }
}
