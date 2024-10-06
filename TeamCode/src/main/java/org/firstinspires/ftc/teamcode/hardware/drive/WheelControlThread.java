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
import java.util.ArrayList;
import org.firstinspires.ftc.lib.trobotix.Motor;
import org.firstinspires.ftc.lib.trobotix.TelemetryThread;
import org.firstinspires.ftc.lib.trobotix.Utils;
import org.firstinspires.ftc.lib.trobotix.controller.SimplePIDFController;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class WheelControlThread extends Thread {
  private final double[] wheelSpeeds = new double[4];

  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final SimplePIDFController frontLeftDriveController;
  private final SimplePIDFController frontRightDriveController;
  private final SimplePIDFController backLeftDriveController;
  private final SimplePIDFController backRightDriveController;

  public WheelControlThread(OpMode opMode) {
    frontLeft = new Motor(opMode, "frontLeft");
    frontRight = new Motor(opMode, "frontRight");
    backLeft = new Motor(opMode, "backLeft");
    backRight = new Motor(opMode, "backRight");

    frontLeft.setInverted(false);
    backLeft.setInverted(false);
    frontRight.setInverted(true);
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

    setDaemon(true);
  }

  @Override
  public void run() {
    while (Utils.opModeActiveSupplier.getAsBoolean()) {
      var startTime = Utils.getTimeSeconds();
      // Read data
      double[] speeds;
      synchronized (wheelSpeeds) {
        speeds = wheelSpeeds.clone();
      }
      frontLeft.setVoltage(frontLeftDriveController.calculate(frontLeft.getVelocity(), speeds[0]));
      frontRight.setVoltage(
          frontRightDriveController.calculate(frontRight.getVelocity(), speeds[1]));
      backLeft.setVoltage(backLeftDriveController.calculate(backLeft.getVelocity(), speeds[2]));
      backRight.setVoltage(backRightDriveController.calculate(backRight.getVelocity(), speeds[3]));
      var dT = Utils.getTimeSeconds() - startTime;
      TelemetryThread.addData("WheelControlThread/Timing (ms)", dT * 1000);
      TelemetryThread.addData("WheelControlThread/Frequency", 1.0 / dT);
    }
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    synchronized (this.wheelSpeeds) {
      this.wheelSpeeds[0] = wheelSpeeds.frontLeftMetersPerSecond;
      this.wheelSpeeds[1] = wheelSpeeds.frontRightMetersPerSecond;
      this.wheelSpeeds[2] = wheelSpeeds.rearLeftMetersPerSecond;
      this.wheelSpeeds[3] = wheelSpeeds.rearRightMetersPerSecond;
    }
  }
}
