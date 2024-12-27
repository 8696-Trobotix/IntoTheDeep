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
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide extends SubsystemBase {
  private final Motor motor;
  private final ViperSlideFeedforward velocityFF;
  private final double velocityP;
  private final PIDController positionPID;

  private final double maxVelMmPerSec;

  private final Telemetry telemetry;

  public final double minPosMm = 378;
  public final double maxPosMm = 1000;

  public Slide(BaseOpMode opMode) {
    motor = new Motor(opMode, "linearSlide");
    telemetry = opMode.telemetry;

    double pulleyDiameterMm = Units.inchesToMeters(1.5) * 1000;
    motor.setConversionFactor(537.689839572 / (pulleyDiameterMm * Math.PI));
    motor.setInverted(false);
    motor.setPosition(minPosMm);

    var kV = 12 / (Units.rotationsPerMinuteToRadiansPerSecond(312) * (pulleyDiameterMm / 2));
    velocityFF = new ViperSlideFeedforward(minPosMm, maxPosMm, .3, .7, .95, .55, kV);
    maxVelMmPerSec =
        (12
                - Math.max(
                    velocityFF.kS_bottom + velocityFF.kG_bottom,
                    velocityFF.kS_top + velocityFF.kG_top))
            / kV;
    velocityP = .5 * kV;
    positionPID = new PIDController(10, 0, 0);
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
    return run(() -> runPosition(positionMm));
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

    var voltage =
        velocityFF.calculate(motor.getPosition(), targetVel)
            + velocityP * (targetVel - motor.getVelocity());
    telemetry.addData("Slide/Voltage", voltage);
    motor.setVoltage(voltage);
  }

  private void runPosition(double targetPosition) {
    runVel(positionPID.calculate(motor.getPosition(), targetPosition));
  }

  public Command runVoltage(DoubleSupplier voltageSupplier) {
    return run(() -> motor.setVoltage(voltageSupplier.getAsDouble()));
  }
}
