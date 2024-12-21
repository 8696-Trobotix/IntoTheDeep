// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.slide;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.controller.ElevatorFeedforward;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide extends SubsystemBase {
  private final Motor motor;
  private final ElevatorFeedforward velocityFF;
  private final double velocityP;
  private final PIDController positionPID;

  private final double maxVelMmPerSec;

  private final Telemetry telemetry;

  public Slide(BaseOpMode opMode) {
    motor = new Motor(opMode, "linearSlide");
    telemetry = opMode.telemetry;

    double pulleyDiameterMm = 32.25;
    motor.setConversionFactor(537.689839572 / (pulleyDiameterMm * Math.PI));
    motor.setInverted(true);

    maxVelMmPerSec = Units.rotationsPerMinuteToRadiansPerSecond(312) * (pulleyDiameterMm / 2);
    velocityFF = new ElevatorFeedforward(0.77, 0.31, 12 / maxVelMmPerSec);
    velocityP = 0;
    positionPID = new PIDController(5, 0, 0);
  }

  @Override
  public void periodic() {
    telemetry.addData("Slide/Position", motor.getPosition());
  }

  public Command teleopControl(DoubleSupplier control) {
    return run(() -> {
      telemetry.addData("Gamepad2/control",control.getAsDouble());
      telemetry.addData("Slide/maxVel", maxVelMmPerSec);
      runVel(control.getAsDouble() * maxVelMmPerSec / 4);
    });
  }

  private void runVel(double targetVel) {
    var currentVel = motor.getVelocity();

    if (motor.getPosition() <= 0 && targetVel < 0) {
      targetVel = 0;
    }
    targetVel = MathUtil.clamp(targetVel, -maxVelMmPerSec / 4, maxVelMmPerSec / 4);
    telemetry.addData("Slide/Target vel", targetVel);
    telemetry.addData("Slide/Actual vel", currentVel);

    var voltage = velocityFF.calculate(targetVel) + velocityP * (targetVel - currentVel);
    telemetry.addData("Slide/Voltage", voltage);
    motor.setVoltage(voltage
        );
  }

  private void runPosition(double targetPosition) {
    targetPosition = MathUtil.clamp(targetPosition, 0, 10);
    double currentPos = motor.getPosition();

    runVel(positionPID.calculate(currentPos, targetPosition));
  }

  public Command runVoltage(DoubleSupplier voltageSupplier) {
    return run(() -> motor.setVoltage(voltageSupplier.getAsDouble()));
  }
}
