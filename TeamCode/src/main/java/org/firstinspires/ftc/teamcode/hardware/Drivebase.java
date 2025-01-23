// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.sequence;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.hardware.Gyro;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.trobotix.hardware.RelativeEncoder;
import org.firstinspires.ftc.lib.trobotix.kinematics.OdometryPodKinematics;
import org.firstinspires.ftc.lib.trobotix.kinematics.OdometryPodWheelPositions;
import org.firstinspires.ftc.lib.trobotix.kinematics.OdometryPods;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.Commands;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.controller.SimpleMotorFeedforward;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivebase extends SubsystemBase {
  private final MecanumDriveKinematics kinematics;
  private final OdometryPods odometry;
  private final OdometryPodKinematics podKinematics;
  private final double maxStrafePodSpeed;
  private final Gyro gyro;

  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final SimpleMotorFeedforward driveController;

  private final RelativeEncoder[] encoders = new RelativeEncoder[3];

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  private final Telemetry telemetry;

  public Drivebase(BaseOpMode opMode) {
    this(opMode, false);
  }

  public Drivebase(BaseOpMode opMode, boolean resetGyro) {
    double trackLength = Units.inchesToMeters(18);
    double trackWidth = Units.inchesToMeters(18);
    kinematics =
        new MecanumDriveKinematics(
            new Translation2d(trackLength / 2, trackWidth / 2),
            new Translation2d(trackLength / 2, -trackWidth / 2),
            new Translation2d(-trackLength / 2, trackWidth / 2),
            new Translation2d(-trackLength / 2, -trackWidth / 2));

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

    double driveWheelDiameter = 104.0 / 1000.0;

    var driveMotor = DCMotor.getGoBILDA5203_0019(1);

    topTranslationalSpeedMetersPerSec = (driveWheelDiameter / 2) * driveMotor.freeSpeedRadPerSec;
    driveController = new SimpleMotorFeedforward(.85, 12 / topTranslationalSpeedMetersPerSec);
    topAngularSpeedRadPerSec =
        topTranslationalSpeedMetersPerSec / Math.hypot(trackWidth / 2, trackLength / 2);

    double podTicksPerRotation = 8192;
    double podWheelCircumference = (35.0 / 1000) * Math.PI;
    encoders[0] =
        new RelativeEncoder(opMode, "leftPod", false, podTicksPerRotation / podWheelCircumference);
    encoders[1] =
        new RelativeEncoder(opMode, "rightPod", false, podTicksPerRotation / podWheelCircumference);
    encoders[2] =
        new RelativeEncoder(opMode, "backPod", false, podTicksPerRotation / podWheelCircumference);
    double minMicrosecondsPerTick = 18;
    double maxTicksPerSecond = 1.0 / (minMicrosecondsPerTick / 1e6);
    maxStrafePodSpeed = (maxTicksPerSecond / podTicksPerRotation) * podWheelCircumference;

    // +X = forwards
    // +Y = left
    // CCW+
    podKinematics =
        new OdometryPodKinematics(
            new Transform2d(.0275, 0.0775 / 2, Rotation2d.kZero),
            new Transform2d(.0275, -0.0725, Rotation2d.kZero),
            new Transform2d(-.07, 0.05, Rotation2d.kCCW_90deg));
    odometry = new OdometryPods(podKinematics);

    gyro =
        new Gyro(
            opMode,
            "IMU",
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP),
            resetGyro);

    xController = new PIDController(4);
    yController = new PIDController(4);
    yawController = new PIDController(2);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(.02, .05);
    yController.setTolerance(.02, .05);
    yawController.setTolerance(Units.degreesToRadians(10), Units.degreesToRadians(10));

    this.telemetry = opMode.telemetry;
  }

  private final double topTranslationalSpeedMetersPerSec;
  private final double topAngularSpeedRadPerSec;

  @Override
  public void periodic() {
    var leftPodPos = encoders[0].getPosition();
    var rightPodPos = encoders[1].getPosition();
    var backPodPos = encoders[2].getPosition();

    telemetry.addData("Drivebase/leftPodPos", leftPodPos);
    telemetry.addData("Drivebase/rightPodPos", rightPodPos);
    telemetry.addData("Drivebase/backPodPos", backPodPos);

    var pose =
        odometry.update(
            gyro.getYaw(), new OdometryPodWheelPositions(leftPodPos, rightPodPos, backPodPos));

    telemetry.addData("Drivebase/Odo X", pose.getX());
    telemetry.addData("Drivebase/Odo Y", pose.getY());
    telemetry.addData("Drivebase/Odo Yaw", pose.getRotation().getDegrees());
  }

  private void robotRelativeDrive(ChassisSpeeds chassisSpeeds) {
    var speeds = kinematics.toWheelSpeeds(chassisSpeeds);
    speeds.desaturate(topTranslationalSpeedMetersPerSec);
    var desaturatedChassisSpeeds = kinematics.toChassisSpeeds(speeds);
    telemetry.addData("Drivebase/Commanded vel X", desaturatedChassisSpeeds.vxMetersPerSecond);
    telemetry.addData("Drivebase/Commanded vel Y", desaturatedChassisSpeeds.vyMetersPerSecond);
    telemetry.addData("Drivebase/Commanded omega", desaturatedChassisSpeeds.omegaRadiansPerSecond);

    frontLeft.set(driveController.calculate(speeds.frontLeftMetersPerSecond));
    frontRight.set(driveController.calculate(speeds.frontRightMetersPerSecond));
    backLeft.set(driveController.calculate(speeds.rearLeftMetersPerSecond));
    backRight.set(driveController.calculate(speeds.rearRightMetersPerSecond));
  }

  public Command alignToPose(Pose2d pose) {
    return sequence(
        runOnce(
            () -> {
              xController.reset();
              yController.reset();
              yawController.reset();
            }),
        run(() -> {
              var fieldRelativeSpeeds =
                  new ChassisSpeeds(
                      MathUtil.clamp(
                          xController.calculate(odometry.getPoseMeters().getX(), pose.getX()),
                          -topTranslationalSpeedMetersPerSec * speedMult,
                          topTranslationalSpeedMetersPerSec * speedMult),
                      MathUtil.clamp(
                          yController.calculate(odometry.getPoseMeters().getY(), pose.getY()),
                          -topTranslationalSpeedMetersPerSec * speedMult,
                          topTranslationalSpeedMetersPerSec * speedMult),
                      MathUtil.clamp(
                          yawController.calculate(
                              odometry.getPoseMeters().getRotation().getRadians(),
                              pose.getRotation().getRadians()),
                          -topTranslationalSpeedMetersPerSec * speedMult,
                          topTranslationalSpeedMetersPerSec * speedMult));
              var robotRelativeSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, gyro.getYaw());

              // Limit strafing speed to prevent pod drift in that direction
              var podSpeeds = podKinematics.toWheelSpeeds(robotRelativeSpeeds);
              if (Math.abs(podSpeeds.podSpeeds[2]) > maxStrafePodSpeed) {
                podSpeeds.podSpeeds[2] = Math.copySign(maxStrafePodSpeed, podSpeeds.podSpeeds[2]);
                robotRelativeSpeeds = podKinematics.toChassisSpeeds(podSpeeds);
              }

              robotRelativeDrive(robotRelativeSpeeds);

              telemetry.addData("PID/X Error", pose.getX() - odometry.getPoseMeters().getX());
              telemetry.addData("PID/Y Error", pose.getY() - odometry.getPoseMeters().getY());
              telemetry.addData(
                  "PID/Yaw Error",
                  pose.getRotation().minus(odometry.getPoseMeters().getRotation()).getDegrees());
            })
            .until(
                () ->
                    xController.atSetpoint()
                        && yController.atSetpoint()
                        && yawController.atSetpoint())
            .finallyDo(() -> robotRelativeDrive(new ChassisSpeeds())));
  }

  public Command teleopDrive(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput) {
    return run(
        () -> {
          var xControl = xInput.getAsDouble();
          var yControl = yInput.getAsDouble();
          var omegaControl = omegaInput.getAsDouble();

          var controlMagnitude = Math.hypot(xControl, yControl);
          if (controlMagnitude > 1) {
            xControl /= controlMagnitude;
            yControl /= controlMagnitude;
          } else {
            xControl *= controlMagnitude;
            yControl *= controlMagnitude;
          }

          omegaControl *= Math.abs(omegaControl);
          robotRelativeDrive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      xControl * topTranslationalSpeedMetersPerSec * speedMult,
                      yControl * topTranslationalSpeedMetersPerSec * speedMult,
                      omegaControl * topAngularSpeedRadPerSec * speedMult),
                  gyro.getYaw()));
        });
  }

  public Command alignHumanPlayer(DoubleSupplier xInput, DoubleSupplier yInput) {
    return teleopDrive(
        xInput, yInput, () -> yawController.calculate(gyro.getYaw().getRadians(), Math.PI));
  }

  public Command alignSpecimen(DoubleSupplier xInput, DoubleSupplier yInput) {
    return teleopDrive(
        xInput, yInput, () -> yawController.calculate(gyro.getYaw().getRadians(), 0));
  }

  private double speedMult = .4;

  public Command setSpeedMult(double speedMult) {
    return Commands.runOnce(() -> this.speedMult = speedMult);
  }

  public Command driveVel(ChassisSpeeds speeds) {
    return run(() -> robotRelativeDrive(speeds))
        .finallyDo(() -> robotRelativeDrive(new ChassisSpeeds()));
  }

  public Command setPosition(Pose2d pose) {
    return runOnce(
        () ->
            odometry.resetPosition(
                gyro.getYaw(),
                new OdometryPodWheelPositions(
                    encoders[0].getPosition(),
                    encoders[1].getPosition(),
                    encoders[2].getPosition()),
                pose));
  }

  public Command runVoltage(DoubleSupplier voltageSupplier) {
    return run(
        () -> {
          var voltage = voltageSupplier.getAsDouble();
          frontLeft.set(voltage);
          frontRight.set(voltage);
          backLeft.set(voltage);
          backRight.set(voltage);
        });
  }
}
