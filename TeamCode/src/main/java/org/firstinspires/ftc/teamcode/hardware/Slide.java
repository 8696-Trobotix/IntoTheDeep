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

  public final double minPosMm = 145;
  public final double maxPosMm = 950;

  public Slide(BaseOpMode opMode) {
    motor = new Motor(opMode, "linearSlide");
    telemetry = opMode.telemetry;

    double pulleyDiameterMm = Units.inchesToMeters(1.5) * 1000;
    motor.setConversionFactor(537.689839572 / (pulleyDiameterMm * Math.PI));
    motor.setInverted(false);
    motor.setPosition(minPosMm);

    maxVelMmPerSec = Units.rotationsPerMinuteToRadiansPerSecond(312) * (pulleyDiameterMm / 2);
    velocityFF =
        new ViperSlideFeedforward(
            minPosMm, maxPosMm, 0.78, 0.27, 0.635, 0.785, 12 / maxVelMmPerSec);
    velocityP = 0;
    positionPID = new PIDController(5, 0, 0);
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
          runVel(motor.getPosition(), control.getAsDouble() * maxVelMmPerSec);
        });
  }

  public Command goToPosition(double positionMm) {
    return run(() -> runPosition(positionMm));
  }

  private void runVel(double currentPos, double targetVel) {
    if ((currentPos < minPosMm && targetVel < 0) || (currentPos > maxPosMm && targetVel > 0)) {
      targetVel = 0;
    }
    targetVel = MathUtil.clamp(targetVel, -maxVelMmPerSec, maxVelMmPerSec);
    telemetry.addData("Slide/Target vel", targetVel);

    var voltage = velocityFF.calculate(currentPos, targetVel);
    telemetry.addData("Slide/Voltage", voltage);
    motor.setVoltage(voltage);
  }

  private void runPosition(double targetPosition) {
    double currentPos = motor.getPosition();

    runVel(currentPos, positionPID.calculate(currentPos, targetPosition));
  }

  public Command runVoltage(DoubleSupplier voltageSupplier) {
    return run(() -> motor.setVoltage(voltageSupplier.getAsDouble()));
  }
}
