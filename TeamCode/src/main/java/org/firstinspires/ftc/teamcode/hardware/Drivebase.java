// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.Utils;
import org.firstinspires.ftc.lib.trobotix.controller.SimplePIDFController;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.trobotix.hardware.RelativeEncoder;
import org.firstinspires.ftc.lib.trobotix.kinematics.OdometryPodWheelPositions;
import org.firstinspires.ftc.lib.trobotix.kinematics.OdometryPods;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.Subsystem;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.filter.SlewRateLimiter;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivebase implements Subsystem {
  private final MecanumDriveKinematics kinematics;
  private final OdometryPods odometry;

  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final SimplePIDFController frontLeftDriveController;
  private final SimplePIDFController frontRightDriveController;
  private final SimplePIDFController backLeftDriveController;
  private final SimplePIDFController backRightDriveController;

  private final RelativeEncoder[] encoders = new RelativeEncoder[3];

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  private final Telemetry telemetry;

  public Drivebase(OpMode opMode) {
    kinematics =
        new MecanumDriveKinematics(
            WHEEL_POSITIONS[0], WHEEL_POSITIONS[1], WHEEL_POSITIONS[2], WHEEL_POSITIONS[3]);

    frontLeft = new Motor(opMode, "frontLeft"); // 1
    frontRight = new Motor(opMode, "frontRight"); // 3
    backLeft = new Motor(opMode, "backLeft"); // 0
    backRight = new Motor(opMode, "backRight"); // 2

    frontLeft.setInverted(false);
    backLeft.setInverted(true);
    frontRight.setInverted(true);
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

    encoders[0] =
        new RelativeEncoder(opMode, "leftPod", false, 8192 / Units.inchesToMeters(Math.PI));
    encoders[1] =
        new RelativeEncoder(opMode, "rightPod", false, 8192 / Units.inchesToMeters(Math.PI));
    encoders[2] =
        new RelativeEncoder(opMode, "backPod", false, 8192 / Units.inchesToMeters(Math.PI));

    // +X = forwards
    // +Y = left
    // CCW+
    odometry =
        new OdometryPods(
            new Transform2d(.0275, 0.0775 / 2, Rotation2d.kZero),
            new Transform2d(.0275, -0.0725, Rotation2d.kZero),
            new Transform2d(-.07, 0.025, Rotation2d.kCCW_90deg));

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    this.telemetry = opMode.telemetry;
  }

  private final double topTranslationalSpeedMetersPerSec =
      Utils.minimum(
              FRONT_LEFT_WHEEL_DIAMETER,
              FRONT_RIGHT_WHEEL_DIAMETER,
              BACK_LEFT_WHEEL_DIAMETER,
              BACK_RIGHT_WHEEL_DIAMETER)
          * Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
          / 2;
  private final double topAngularSpeedRadPerSec =
      topTranslationalSpeedMetersPerSec / Math.hypot(TRACK_LENGTH / 2, TRACK_WIDTH / 2);

  @Override
  public void periodic() {
    var leftPodPos = encoders[0].getPosition();
    var rightPodPos = encoders[1].getPosition();
    var backPodPos = encoders[2].getPosition();

    telemetry.addData("Drivebase/leftPodPos", leftPodPos);
    telemetry.addData("Drivebase/rightPodPos", rightPodPos);
    telemetry.addData("Drivebase/backPodPos", backPodPos);

    var pose = odometry.update(new OdometryPodWheelPositions(leftPodPos, rightPodPos, backPodPos));

    telemetry.addData("Drivebase/Odo X", pose.getX());
    telemetry.addData("Drivebase/Odo Y", pose.getY());
    telemetry.addData("Drivebase/Odo Yaw", pose.getRotation().getDegrees());
  }

  private void drive(ChassisSpeeds chassisSpeeds) {
    var speeds = kinematics.toWheelSpeeds(chassisSpeeds);
    speeds.desaturate(topTranslationalSpeedMetersPerSec);

    frontLeft.setVoltage(
        frontLeftDriveController.calculate(
            frontLeft.getVelocity(), speeds.frontLeftMetersPerSecond));
    frontRight.setVoltage(
        frontRightDriveController.calculate(
            frontRight.getVelocity(), speeds.frontRightMetersPerSecond));
    backLeft.setVoltage(
        backLeftDriveController.calculate(backLeft.getVelocity(), speeds.rearLeftMetersPerSecond));
    backRight.setVoltage(
        backRightDriveController.calculate(
            backRight.getVelocity(), speeds.rearRightMetersPerSecond));
  }

  public Command alignToPose(Pose2d pose) {
    return run(
        () ->
            drive(
                new ChassisSpeeds(
                    xController.calculate(odometry.getPoseMeters().getX(), pose.getX()),
                    yController.calculate(odometry.getPoseMeters().getY(), pose.getY()),
                    yawController.calculate(
                        odometry.getPoseMeters().getRotation().getRadians(),
                        pose.getRotation().getRadians()))));
  }

  private final double ZERO_TO_FULL_TIME = .125;
  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(topTranslationalSpeedMetersPerSec / ZERO_TO_FULL_TIME);
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(topTranslationalSpeedMetersPerSec / ZERO_TO_FULL_TIME);
  private final SlewRateLimiter steerLimiter =
      new SlewRateLimiter(topAngularSpeedRadPerSec / ZERO_TO_FULL_TIME);

  public Command teleopDrive(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput) {
    return run(
        () ->
            drive(
                new ChassisSpeeds(
                    xLimiter.calculate(xInput.getAsDouble() * topTranslationalSpeedMetersPerSec),
                    yLimiter.calculate(yInput.getAsDouble() * topTranslationalSpeedMetersPerSec),
                    steerLimiter.calculate(omegaInput.getAsDouble() * topAngularSpeedRadPerSec))));
  }

  public Command driveVel(ChassisSpeeds speeds) {
    return run(() -> drive(speeds)).finallyDo(() -> drive(new ChassisSpeeds()));
  }

  public Pose2d getPose() {
    return new Pose2d();
  }
}
