// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.slide;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.controller.ElevatorFeedforward;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.trajectory.ExponentialProfile;

public class Slide extends SubsystemBase {
  private final Motor motor;
  private final ElevatorFeedforward velocityFF;
  private final double velocityP;
  private final ExponentialProfile positionProfile;
  private final PIDController positionPID;

  public Slide(BaseOpMode opMode) {
    motor = new Motor(opMode, "slide");

    // TODO: Configure

    velocityFF = new ElevatorFeedforward(0, 0, 0);
    velocityP = 5;
    positionProfile =
        new ExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(
                12 - velocityFF.getKg() - velocityFF.getKs(),
                velocityFF.getKv(),
                velocityFF.getKa()));
    positionPID = new PIDController(5, 0, 0);
  }

  private void runVel(double targetVel, double voltage, double dt) {
    runVel(motor.getVelocity(), targetVel, voltage, dt);
  }

  private void runVel(double currentVel, double targetVel, double voltage, double dt) {
    double maxAccel = velocityFF.maxAchievableAcceleration(voltage, currentVel);
    double minAccel = velocityFF.minAchievableAcceleration(voltage, currentVel);

    targetVel = MathUtil.clamp(targetVel, minAccel * dt, maxAccel * dt);

    motor.setVoltage(
        velocityFF.calculate(currentVel, targetVel) + velocityP * (targetVel - currentVel));
  }

  private void runPosition(double targetPosition, double voltage, double dt) {
    double currentPos = motor.getPosition();
    double currentVel = motor.getVelocity();

    var state =
        positionProfile.calculate(
            dt,
            new ExponentialProfile.State(currentPos, currentVel),
            new ExponentialProfile.State(targetPosition, 0));

    runVel(
        currentVel,
        state.velocity + positionPID.calculate(currentPos, state.position),
        voltage,
        dt);
  }
}
