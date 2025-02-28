// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.controller.ViperSlideFeedforward;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.Commands;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ScoringElevator extends SubsystemBase {
  private final Motor motor;
  private final ViperSlideFeedforward velocityFF;
  private final PIDController positionPID;

  private final double maxVelMmPerSec;

  private final Telemetry telemetry;

  private final double minPosMm = 240;
  private final double maxPosMm = 862;

  public ScoringElevator(BaseOpMode opMode) {
    motor = new Motor(opMode, "linearSlide", true);
    telemetry = opMode.telemetry;

    double gearRatio = (((1 + (46.0 / 17.0))) * (1 + (46.0 / 17.0)));
    double ticksPerRotation = (28 * gearRatio);
    double distancePerRotation = 2 * 60; // Pitch * teeth
    motor.setConversionFactor(ticksPerRotation / distancePerRotation);
    motor.setInverted(false);
    motor.setPosition(minPosMm);

    var kV = 12 / ((6000.0 / 60.0) / gearRatio * distancePerRotation);
    velocityFF = new ViperSlideFeedforward(minPosMm, maxPosMm, .95, .5, .35, 1.115, kV);
    maxVelMmPerSec =
        (12
                - Math.max(
                    velocityFF.kS_bottom + velocityFF.kG_bottom,
                    velocityFF.kS_top + velocityFF.kG_top))
            / kV;
    positionPID = new PIDController(7.5, 0, 0, opMode.dtSupplier());
    positionPID.setTolerance(10, 5);
  }

  @Override
  public void periodic() {
    telemetry.addData("Slide/Position", motor.getPosition());
  }

  public Command manualControl(DoubleSupplier control) {
    return run(
        () -> {
          telemetry.addData("Gamepad2/control", control.getAsDouble());
          telemetry.addData("Slide/maxVel", maxVelMmPerSec);
          runVel(control.getAsDouble() * maxVelMmPerSec);
        });
  }

  public Command goToPosition(double positionMm) {
    return runOnce(positionPID::reset)
        .andThen(
            run(() -> runPosition(positionMm))
                .until(positionPID::atSetpoint)
                .finallyDo(() -> runVel(0)));
  }

  public Command alignHighSpecimen() {
    return goToPosition(660);
  }

  public Command alignHighSpecimenTeleop() {
    return runOnce(positionPID::reset)
        .andThen(
            run(() -> runPosition(660)));
  }

  public Command scoreHighSpecimen() {
    return goToPosition(525);
  }

  public Command retract() {
    return goToPosition(minPosMm);
  }

  public Command retractDefault() {
    return runOnce(positionPID::reset).andThen(run(() -> runPosition(minPosMm)));
  }

  private void runVel(double targetVel) {
    telemetry.addData("Slide/Override enabled", override);
    if (!override
        && ((motor.getPosition() < minPosMm && targetVel < 0)
            || (motor.getPosition() > maxPosMm && targetVel > 0))) {
      targetVel = 0;
    }
    targetVel =
        MathUtil.clamp(
            targetVel,
            velocityFF.getMinAchievableVel(motor.getPosition()),
            velocityFF.getMaxAchievableVel(motor.getPosition()));
    telemetry.addData("Slide/Target vel", targetVel);
    telemetry.addData("Slide/Actual vel", motor.getVelocity());

    var voltage = velocityFF.calculate(motor.getPosition(), targetVel);
    telemetry.addData("Slide/Voltage", voltage);
    motor.set(voltage);
  }

  private boolean override = false;

  public Command enableOverride() {
    return Commands.runOnce(() -> override = true);
  }

  public Command disableOverride() {
    return Commands.runOnce(
        () -> {
          override = false;
          motor.setPosition(minPosMm);
        });
  }

  private void runPosition(double targetPosition) {
    telemetry.addData("Slide/Target pos", targetPosition);
    runVel(positionPID.calculate(motor.getPosition(), targetPosition));
  }

  public Command runVoltage(DoubleSupplier voltageSupplier) {
    return run(() -> motor.set(voltageSupplier.getAsDouble()));
  }
}
