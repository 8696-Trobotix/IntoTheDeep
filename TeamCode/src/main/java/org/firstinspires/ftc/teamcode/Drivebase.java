// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.wpilib.math.utils.Units;

public class Drivebase {
  private final PhotonDcMotor frontLeft;
  private final PhotonDcMotor frontRight;
  private final PhotonDcMotor backLeft;
  private final PhotonDcMotor backRight;

  private final MecanumDriveKinematics kinematics;
  private final MecanumDriveOdometry odometry;

  private final PIDFController frontLeftDriveController;
  private final PIDFController frontRightDriveController;
  private final PIDFController backLeftDriveController;
  private final PIDFController backRightDriveController;

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
    odometry = new MecanumDriveOdometry(kinematics, new Rotation2d(), new Pose2d());

    frontLeftDriveController =
        new PIDFController(
            0,
            0,
            0,
            Units.rotationsPerMinuteToRadiansPerSecond(Constants.Drivebase.DRIVE_MOTOR_MAX_RPM)
                * Constants.Drivebase.FRONT_LEFT_WHEEL_DIAMETER);
    frontRightDriveController =
        new PIDFController(
            0,
            0,
            0,
            Units.rotationsPerMinuteToRadiansPerSecond(Constants.Drivebase.DRIVE_MOTOR_MAX_RPM)
                * Constants.Drivebase.FRONT_RIGHT_WHEEL_DIAMETER);
    backLeftDriveController =
        new PIDFController(
            0,
            0,
            0,
            Units.rotationsPerMinuteToRadiansPerSecond(Constants.Drivebase.DRIVE_MOTOR_MAX_RPM)
                * Constants.Drivebase.BACK_LEFT_WHEEL_DIAMETER);
    backRightDriveController =
        new PIDFController(
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

  public void drive(ChassisSpeeds chassisSpeeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    double frontLeftVel =
        Constants.Drivebase.FRONT_LEFT_WHEEL_DIAMETER
            * Units.rotationsToRadians(
                frontLeft.getVelocity()
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
  }
}
