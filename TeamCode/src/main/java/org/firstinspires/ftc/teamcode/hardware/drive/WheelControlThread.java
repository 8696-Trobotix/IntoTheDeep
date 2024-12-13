// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.drive;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.BACK_LEFT_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.BACK_RIGHT_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.DRIVE_MOTOR_MAX_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.FRONT_LEFT_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.FRONT_RIGHT_WHEEL_DIAMETER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.lib.trobotix.EndableThread;
import org.firstinspires.ftc.lib.trobotix.controller.SimplePIDFController;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.trobotix.hardware.RelativeEncoder;
import org.firstinspires.ftc.lib.trobotix.kinematics.OmniWheelOdometryNoGyro;
import org.firstinspires.ftc.lib.trobotix.kinematics.OmniWheelPositions;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class WheelControlThread extends EndableThread {
  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final SimplePIDFController frontLeftDriveController;
  private final SimplePIDFController frontRightDriveController;
  private final SimplePIDFController backLeftDriveController;
  private final SimplePIDFController backRightDriveController;

  private final OmniWheelOdometryNoGyro odometry =
      new OmniWheelOdometryNoGyro(
          new Transform2d(.1, 0, Rotation2d.kZero),
          new Transform2d(-.1, 0, Rotation2d.kZero),
          new Transform2d(0, -.1, Rotation2d.kCCW_90deg));
  private final RelativeEncoder[] encoders = new RelativeEncoder[3];

  public WheelControlThread(OpMode opMode) {
    super("Wheel Control Thread");
    frontLeft = new Motor(opMode, "frontLeft"); // 1
    frontRight = new Motor(opMode, "frontRight"); // 3
    backLeft = new Motor(opMode, "backLeft"); // 0
    backRight = new Motor(opMode, "backRight"); // 2

    frontLeft.setInverted(true);
    backLeft.setInverted(false);
    frontRight.setInverted(false);
    backRight.setInverted(true);

    frontLeft.setIdleBrake(true);
    frontRight.setIdleBrake(true);
    backLeft.setIdleBrake(true);
    backRight.setIdleBrake(true);

    // Convert from encoder ticks to meters
    // Distance per rotation (meters) / encoder ticks per rotation
    // The above has to be flipped over since it's a divide not a multiply for conversion factor
    frontLeft.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (FRONT_LEFT_WHEEL_DIAMETER * Math.PI));
    frontRight.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (FRONT_RIGHT_WHEEL_DIAMETER * Math.PI));
    backLeft.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (BACK_LEFT_WHEEL_DIAMETER * Math.PI));
    backRight.setConversionFactor(
        DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION / (BACK_RIGHT_WHEEL_DIAMETER * Math.PI));

    frontLeftDriveController =
        new SimplePIDFController(
            0
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_LEFT_WHEEL_DIAMETER
                    / 2),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_LEFT_WHEEL_DIAMETER
                    / 2));
    frontRightDriveController =
        new SimplePIDFController(
            0
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_RIGHT_WHEEL_DIAMETER
                    / 2),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_RIGHT_WHEEL_DIAMETER
                    / 2));
    backLeftDriveController =
        new SimplePIDFController(
            0
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_LEFT_WHEEL_DIAMETER
                    / 2),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_LEFT_WHEEL_DIAMETER
                    / 2));
    backRightDriveController =
        new SimplePIDFController(
            0
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_RIGHT_WHEEL_DIAMETER
                    / 2),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_RIGHT_WHEEL_DIAMETER
                    / 2));

    encoders[0] = new RelativeEncoder(opMode, "leftEncoder", 8192 / Units.inchesToMeters(1));
    encoders[1] = new RelativeEncoder(opMode, "rightEncoder", 8192 / Units.inchesToMeters(1));
    encoders[2] = new RelativeEncoder(opMode, "backEncoder", 8192 / Units.inchesToMeters(1));
  }

  private final double[] wheelSpeeds = new double[4];

  @Override
  public void loop() {
    // Read data
    double[] speeds;
    synchronized (wheelSpeeds) {
      speeds = wheelSpeeds.clone();
    }
    frontLeft.setVoltage(frontLeftDriveController.calculate(frontLeft.getVelocity(), speeds[0]));
    frontRight.setVoltage(frontRightDriveController.calculate(frontRight.getVelocity(), speeds[1]));
    backLeft.setVoltage(backLeftDriveController.calculate(backLeft.getVelocity(), speeds[2]));
    backRight.setVoltage(backRightDriveController.calculate(backRight.getVelocity(), speeds[3]));

    var wheelPositions =
        new OmniWheelPositions(
            encoders[0].getPosition(), encoders[1].getPosition(), encoders[2].getPosition());
    synchronized (odometry) {
      odometry.update(wheelPositions);
    }
  }

  void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    synchronized (this.wheelSpeeds) {
      this.wheelSpeeds[0] = wheelSpeeds.frontLeftMetersPerSecond;
      this.wheelSpeeds[1] = wheelSpeeds.frontRightMetersPerSecond;
      this.wheelSpeeds[2] = wheelSpeeds.rearLeftMetersPerSecond;
      this.wheelSpeeds[3] = wheelSpeeds.rearRightMetersPerSecond;
    }
  }

  Pose2d getPose() {
    synchronized (odometry) {
      return odometry.getPoseMeters();
    }
  }
}
