// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.lib.teamlib.Motor;
import org.firstinspires.ftc.teamcode.lib.teamlib.Utils;
import org.firstinspires.ftc.teamcode.lib.teamlib.controller.SimplePIDFController;
import org.firstinspires.ftc.teamcode.lib.teamlib.estimator.OmniWheelPoseEstimator;
import org.firstinspires.ftc.teamcode.lib.teamlib.kinematics.OmniWheelKinematics;
import org.firstinspires.ftc.teamcode.lib.teamlib.kinematics.OmniWheelPositions;
import org.firstinspires.ftc.teamcode.lib.wpilib.commands.Command;
import org.firstinspires.ftc.teamcode.lib.wpilib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.utils.Units;

public class Drivebase implements Subsystem {
  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final MecanumDriveKinematics kinematics;
  private final OmniWheelPoseEstimator odometry;

  private final SimplePIDFController frontLeftDriveController;
  private final SimplePIDFController frontRightDriveController;
  private final SimplePIDFController backLeftDriveController;
  private final SimplePIDFController backRightDriveController;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

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
    // The above has to be flipped over since it's a divide not a multiply for conversion factor
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
        new OmniWheelPoseEstimator(
            new OmniWheelKinematics(new Transform2d[] {}),
            new Rotation2d(),
            new OmniWheelPositions(),
            new Pose2d());

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

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // TODO: Implement sensors
    odometry.update(new Rotation2d(), new OmniWheelPositions());
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
  }

  private void fieldRelativeDrive(ChassisSpeeds chassisSpeeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, new Rotation2d()));
  }

  private void alignToPose(Pose2d pose) {
    pose = Utils.flipAllianceOnRed(pose);

    fieldRelativeDrive(
        new ChassisSpeeds(
            xController.calculate(odometry.getEstimatedPosition().getX(), pose.getX()),
            yController.calculate(odometry.getEstimatedPosition().getY(), pose.getY()),
            yawController.calculate(
                odometry.getEstimatedPosition().getRotation().getRadians(),
                pose.getRotation().getRadians())));
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
          double submersibleWidth = Units.inchesToMeters(30);
          double minY = Units.inchesToMeters(Utils.FIELD_SIZE - submersibleWidth) / 2;
          double maxY = Utils.FIELD_SIZE - minY;

          Pose2d currentPose = Utils.flipAllianceOnRed(odometry.getEstimatedPosition());

          double targetX;
          if (minY < currentPose.getY() && currentPose.getY() < maxY) {
            targetX = 34;
          } else {
            targetX = 24;
          }
          double targetY = MathUtil.interpolate(minY, maxY, (strafeSupplier.getAsDouble() + 1) / 2);

          alignToPose(new Pose2d(targetX, targetY, new Rotation2d()));
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
    odometry.addVisionMeasurement(
        estimatedPose,
        timestamp,
        VecBuilder.fill(translationalStDev, translationalStDev, angularStDev));
  }
}
