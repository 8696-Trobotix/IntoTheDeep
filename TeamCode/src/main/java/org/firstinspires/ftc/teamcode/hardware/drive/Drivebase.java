// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.drive;

import static org.firstinspires.ftc.teamcode.Constants.Drivebase.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.Utils;
import org.firstinspires.ftc.lib.trobotix.estimator.OmniWheelPoseEstimator;
import org.firstinspires.ftc.lib.trobotix.hardware.RelativeEncoder;
import org.firstinspires.ftc.lib.trobotix.kinematics.OmniWheelKinematics;
import org.firstinspires.ftc.lib.trobotix.kinematics.OmniWheelPositions;
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

public class Drivebase implements Subsystem {
  private final MecanumDriveKinematics kinematics;

  private final OmniWheelPoseEstimator odometry;
  private final IMU gyro;
  private final RelativeEncoder[] encoders = new RelativeEncoder[3];

  private final WheelControlThread wheelControlThread;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  public Drivebase(OpMode opMode) {
    kinematics =
        new MecanumDriveKinematics(
            WHEEL_POSITIONS[0], WHEEL_POSITIONS[1], WHEEL_POSITIONS[2], WHEEL_POSITIONS[3]);
    odometry =
        new OmniWheelPoseEstimator(
            new OmniWheelKinematics(
                new Transform2d(0, .1, Rotation2d.kZero),
                new Transform2d(0, -.1, Rotation2d.kZero),
                new Transform2d(-.1, 0, Rotation2d.kZero)),
            Rotation2d.kZero,
            new OmniWheelPositions(),
            new Pose2d());
    for (int i = 0; i < 3; i++) {
      encoders[i] = new RelativeEncoder(opMode, "OdoWheel" + i, 8192 / Units.inchesToMeters(1));
    }

    gyro = opMode.hardwareMap.get(IMU.class, "imu");
    gyro.initialize(
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    gyro.resetYaw();

    wheelControlThread = new WheelControlThread(opMode);

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    odometry.update(
        getYaw(),
        new OmniWheelPositions(
            encoders[0].getPosition(), encoders[1].getPosition(), encoders[3].getPosition()));
  }

  private final double topTranslationalSpeedMetersPerSec =
      Utils.minimum(
              FRONT_LEFT_WHEEL_DIAMETER,
              FRONT_RIGHT_WHEEL_DIAMETER,
              BACK_LEFT_WHEEL_DIAMETER,
              BACK_RIGHT_WHEEL_DIAMETER)
          * Units.rotationsPerMinuteToRadiansPerSecond(DRIVE_MOTOR_MAX_RPM)
          / 4;
  private final double topAngularSpeedRadPerSec =
      topTranslationalSpeedMetersPerSec / Math.hypot(TRACK_LENGTH / 2, TRACK_WIDTH / 2);

  public void drive(ChassisSpeeds chassisSpeeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(topTranslationalSpeedMetersPerSec);

    wheelControlThread.setWheelSpeeds(wheelSpeeds);
  }

  public void fieldRelativeDrive(ChassisSpeeds chassisSpeeds) {
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getYaw()));
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

  public Command goToPoint(Pose2d pose) {
    return run(() -> alignToPose(pose));
  }

  private final double ZERO_TO_FULL_TIME = .25;
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
            fieldRelativeDrive(
                new ChassisSpeeds(
                    xLimiter.calculate(xInput.getAsDouble() * topTranslationalSpeedMetersPerSec),
                    yLimiter.calculate(yInput.getAsDouble() * topTranslationalSpeedMetersPerSec),
                    steerLimiter.calculate(omegaInput.getAsDouble() * topAngularSpeedRadPerSec))));
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

  public Pose2d getPose() {
    return new Pose2d();
  }

  private Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getRobotYawPitchRollAngles().getYaw());
  }
}
