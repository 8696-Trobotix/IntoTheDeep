// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.drive;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.BACK_LEFT_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.BACK_RIGHT_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.DRIVE_MOTOR_MAX_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.FRONT_LEFT_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.Constants.Drivebase.FRONT_RIGHT_WHEEL_DIAMETER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.teamlib.Motor;
import org.firstinspires.ftc.teamcode.lib.teamlib.Utils;
import org.firstinspires.ftc.teamcode.lib.teamlib.controller.SimplePIDFController;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.utils.Units;

public class WheelControlThread extends Thread {
  private MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds();

  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final SimplePIDFController frontLeftDriveController;
  private final SimplePIDFController frontRightDriveController;
  private final SimplePIDFController backLeftDriveController;
  private final SimplePIDFController backRightDriveController;

  public WheelControlThread(LinearOpMode opMode) {
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
            24
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_LEFT_WHEEL_DIAMETER),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_LEFT_WHEEL_DIAMETER));
    frontRightDriveController =
        new SimplePIDFController(
            24
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_RIGHT_WHEEL_DIAMETER),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * FRONT_RIGHT_WHEEL_DIAMETER));
    backLeftDriveController =
        new SimplePIDFController(
            24
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_LEFT_WHEEL_DIAMETER),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_LEFT_WHEEL_DIAMETER));
    backRightDriveController =
        new SimplePIDFController(
            24
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_RIGHT_WHEEL_DIAMETER),
            12
                / (Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
                    * BACK_RIGHT_WHEEL_DIAMETER));
  }

  @Override
  public void run() {
    //noinspection InfiniteLoopStatement
    while (true) {
      // Read data
      double[] speeds;
      try {
        Utils.THREAD_LOCK.readLock().lock();
        speeds =
            new double[] {
              wheelSpeeds.frontLeftMetersPerSecond,
              wheelSpeeds.frontRightMetersPerSecond,
              wheelSpeeds.rearLeftMetersPerSecond,
              wheelSpeeds.rearRightMetersPerSecond
            };
      } finally {
        Utils.THREAD_LOCK.readLock().unlock();
      }

      frontLeft.setVoltage(frontLeftDriveController.calculate(frontLeft.getVelocity(), speeds[0]));
      frontRight.setVoltage(
          frontRightDriveController.calculate(frontRight.getVelocity(), speeds[1]));
      backLeft.setVoltage(backLeftDriveController.calculate(backLeft.getVelocity(), speeds[2]));
      backRight.setVoltage(backRightDriveController.calculate(backRight.getVelocity(), speeds[3]));
    }
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    try {
      Utils.THREAD_LOCK.writeLock().lock();
      this.wheelSpeeds = wheelSpeeds;
    } finally {
      Utils.THREAD_LOCK.writeLock().unlock();
    }
  }
}
