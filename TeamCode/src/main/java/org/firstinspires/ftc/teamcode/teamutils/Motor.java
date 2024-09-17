// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.teamutils;

import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.wpilib.math.MathUtil;

/** Wrapper for {@link PhotonDcMotor} to add extra functionality */
public class Motor {
  private final PhotonDcMotor motorInternal;
  private final PhotonLynxVoltageSensor voltageSensor;

  public Motor(OpMode opMode, String name) {
    motorInternal = (PhotonDcMotor) opMode.hardwareMap.dcMotor.get(name);
    voltageSensor = opMode.hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
  }

  public void setInverted(boolean inverted) {
    motorInternal.setDirection(
        inverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
  }

  public void setIdleBrake(boolean brake) {
    motorInternal.setZeroPowerBehavior(
        brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
  }

  public void set(double power) {
    motorInternal.setPower(MathUtil.clamp(power, -1, 1));
  }

  public void setVoltage(double volts) {
    set(volts / voltageSensor.getCachedVoltage());
  }

  public double getCurrentDraw() {
    return motorInternal.getCorrectedCurrent(CurrentUnit.AMPS);
  }

  private double conversionFactor = 1;

  /**
   * Sets the motor's conversion factor.
   *
   * <p>Values > 1 are reductions, so a 4:1 reduction would make a factor of 4.
   *
   * <p>A common use case is to convert from encoder ticks to rotations, with 1 / ticksPerRotation.
   *
   * @param conversionFactor The new conversion factor.
   */
  public void setConversionFactor(double conversionFactor) {
    this.conversionFactor = conversionFactor;
  }

  /**
   * Gets the motor's current position.
   *
   * @return The position. Units are in encoder ticks by default, but can be changed with {@link
   *     Motor#setConversionFactor(double conversionFactor)}.
   */
  public double getPosition() {
    return motorInternal.getCurrentPosition() / conversionFactor;
  }

  private double lastPos = 0;
  private double lastTime = -1;

  // We calculate motor velocity ourselves because REV sucks and only calculates velocity at 20 hz

  /**
   * Gets the motor's current velocity.
   *
   * @return The velocity. Numerator units follow {@link Motor#getPosition()}. Denominator unit is
   *     seconds.
   */
  public double getVelocity() {
    if (lastTime == -1) {
      lastPos = getPosition();
      lastTime = Utils.getTimeSeconds();
      return 0;
    }
    double currentPos = getPosition();
    double currentTime = Utils.getTimeSeconds();
    double velocity = (currentPos - lastPos) / (currentTime - lastTime);
    lastPos = currentPos;
    lastTime = currentTime;
    return velocity;
  }
}
