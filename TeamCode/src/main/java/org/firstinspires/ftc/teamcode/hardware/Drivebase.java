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
import org.firstinspires.ftc.lib.trobotix.kinematics.FollowerWheelKinematics;
import org.firstinspires.ftc.lib.trobotix.kinematics.FollowerWheelOdometry;
import org.firstinspires.ftc.lib.trobotix.kinematics.FollowerWheelPositions;
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
  private final FollowerWheelOdometry odometry;
  private final Gyro gyro;

  private final Motor frontLeft;
  private final Motor frontRight;
  private final Motor backLeft;
  private final Motor backRight;

  private final SimpleMotorFeedforward driveController;

  private final RelativeEncoder frontEncoder;
  private final RelativeEncoder podB;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController yawController;

  private final Telemetry telemetry;

  private final DoubleSupplier dtSupplier;

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

    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontRight.setInverted(false);
    backRight.setInverted(false);

    frontLeft.setIdleBrake(true);
    frontRight.setIdleBrake(true);
    backLeft.setIdleBrake(true);
    backRight.setIdleBrake(true);

    double driveWheelDiameter = 104.0 / 1000.0;

    var driveMotor = DCMotor.getGoBILDA5203_0019(1);

    topTranslationalSpeedMetersPerSec = (driveWheelDiameter / 2) * driveMotor.freeSpeedRadPerSec;
    driveController = new SimpleMotorFeedforward(.7, 12 / topTranslationalSpeedMetersPerSec);
    topAngularSpeedRadPerSec =
        topTranslationalSpeedMetersPerSec / Math.hypot(trackWidth / 2, trackLength / 2);

    double podTicksPerRotation = 2000;
    double podWheelCircumference = (35.0 / 1000) * Math.PI;
    frontEncoder =
        new RelativeEncoder(opMode, "podA", false, podTicksPerRotation / podWheelCircumference);
    podB = new RelativeEncoder(opMode, "podB", false, podTicksPerRotation / podWheelCircumference);

    gyro =
        new Gyro(
            opMode,
            "imu",
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP),
            resetGyro);

    // +X = forwards
    // +Y = left
    // CCW+
    odometry =
        new FollowerWheelOdometry(
            new FollowerWheelKinematics(
                new Transform2d((-.327 / 2) + (25.0 / 1000), 25.0 / 1000, Rotation2d.k180deg),
                new Transform2d((-.327 / 2) + (25.0 / 1000), -25.0 / 1000, Rotation2d.kCCW_90deg)),
            new FollowerWheelPositions(
                gyro.getYaw(), frontEncoder.getPosition(), podB.getPosition()),
            Pose2d.kZero);

    xController = new PIDController(5);
    yController = new PIDController(5);
    yawController = new PIDController(1.5);
    yawController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(.05, .1);
    yController.setTolerance(.02, .05);
    yawController.setTolerance(Units.degreesToRadians(10), Units.degreesToRadians(10));

    this.telemetry = opMode.telemetry;

    dtSupplier = opMode.dtSupplier();
  }

  private final double topTranslationalSpeedMetersPerSec;
  private final double topAngularSpeedRadPerSec;

  @Override
  public void periodic() {
    var frontPodPos = frontEncoder.getPosition();
    var podBPos = podB.getPosition();
    telemetry.addData("Drivebase/frontPodPos", frontPodPos);
    telemetry.addData("Drivebase/podBPos", podBPos);

    var pose = odometry.update(new FollowerWheelPositions(gyro.getYaw(), frontPodPos, podBPos));

    telemetry.addData("Drivebase/Odo X", pose.getX());
    telemetry.addData("Drivebase/Odo Y", pose.getY());
    telemetry.addData("Drivebase/Odo Yaw", pose.getRotation().getDegrees());
  }

  private void robotRelativeDrive(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, dtSupplier.getAsDouble());
    var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(topTranslationalSpeedMetersPerSec);
    var desaturatedChassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    telemetry.addData("Drivebase/Commanded vel X", desaturatedChassisSpeeds.vxMetersPerSecond);
    telemetry.addData("Drivebase/Commanded vel Y", desaturatedChassisSpeeds.vyMetersPerSecond);
    telemetry.addData("Drivebase/Commanded omega", desaturatedChassisSpeeds.omegaRadiansPerSecond);

    frontLeft.set(driveController.calculate(wheelSpeeds.frontLeftMetersPerSecond));
    frontRight.set(driveController.calculate(wheelSpeeds.frontRightMetersPerSecond));
    backLeft.set(driveController.calculate(wheelSpeeds.rearLeftMetersPerSecond));
    backRight.set(driveController.calculate(wheelSpeeds.rearRightMetersPerSecond));
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
              robotRelativeDrive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, gyro.getYaw()));

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
