// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.controller.ViperSlideFeedforward;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide extends SubsystemBase {
  private final Motor motor;
  private final ViperSlideFeedforward velocityFF;
  private final PIDController positionPID;

  private final double maxVelMmPerSec;

  private final Telemetry telemetry;

  private final double minPosMm = 240;
  private final double maxPosMm = 862;

  public Slide(BaseOpMode opMode) {
    motor = new Motor(opMode, "linearSlide");
    telemetry = opMode.telemetry;

    double ticksPerRad = 537.689839572 / (2 * Math.PI);
    double pulleyDiameterMm = Units.inchesToMeters(1.5) * 1000;
    motor.setConversionFactor(ticksPerRad / (pulleyDiameterMm / 2));
    motor.setInverted(false);
    motor.setPosition(minPosMm);

    var kV = 12 / (Units.rotationsPerMinuteToRadiansPerSecond(312) * (pulleyDiameterMm / 2));
    velocityFF = new ViperSlideFeedforward(minPosMm, maxPosMm, .765, .335, .625, 1.175, kV);
    maxVelMmPerSec =
        (12
                - Math.max(
                    velocityFF.kS_bottom + velocityFF.kG_bottom,
                    velocityFF.kS_top + velocityFF.kG_top))
            / kV;
    positionPID = new PIDController(3, 0, 0);
    positionPID.setTolerance(5, 1);
  }

  @Override
  public void periodic() {
    telemetry.addData("Slide/Position", motor.getPosition());
  }

  public Command teleopControl(DoubleSupplier control) {
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
    return goToPosition(750);
  }

  public Command scoreHighSpecimen() {
    return goToPosition(540);
  }

  public Command retract() {
    return goToPosition(minPosMm);
  }

  private void runVel(double targetVel) {
    if ((motor.getPosition() < minPosMm && targetVel < 0)
        || (motor.getPosition() > maxPosMm && targetVel > 0)) {
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

  private void runPosition(double targetPosition) {
    telemetry.addData("Slide/Target pos", targetPosition);
    runVel(positionPID.calculate(motor.getPosition(), targetPosition));
  }

  public Command runVoltage(DoubleSupplier voltageSupplier) {
    return run(() -> motor.set(voltageSupplier.getAsDouble()));
  }
}
