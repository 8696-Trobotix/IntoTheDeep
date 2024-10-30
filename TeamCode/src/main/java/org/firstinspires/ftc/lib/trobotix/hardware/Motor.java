// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.lib.trobotix.Utils;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.filter.LinearFilter;

/** Wrapper for {@link DcMotorEx} for extra functionality and cleaner code. */
public class Motor {
  private final DcMotorEx motorInternal;
  private final LynxVoltageSensor voltageSensor;

  public Motor(OpMode opMode, String name) {
    motorInternal = (DcMotorEx) opMode.hardwareMap.dcMotor.get(name);
    voltageSensor = opMode.hardwareMap.getAll(LynxVoltageSensor.class).iterator().next();

    motorInternal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorInternal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  /**
   * By default, a motor will typically move counterclockwise when positive voltage is sent. (The
   * only exception is Neverest motors which is clockwise positive.)
   *
   * <p>However, that isn't always desired. Sometimes we may want to have the opposite behaviour.
   *
   * @param inverted If the motor is inverted or not.
   */
  public void setInverted(boolean inverted) {
    motorInternal.setDirection(
        inverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
  }

  /**
   * Typically, we want the motor to stop when we stop applying power to it. However, we
   * occasionally want it to keep coasting when no more power is applied.
   *
   * @param brake Whether or not the motor brakes when no power is applied.
   */
  public void setIdleBrake(boolean brake) {
    motorInternal.setZeroPowerBehavior(
        brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
  }

  //  /**
  //   * Sets the tolerance for which setting new power values is ignored.
  //   *
  //   * <p>It's a waste of processor time to tell the motor to move at .9 power and immediately
  // tell it
  //   * to move at .901 power, so we use the cache tolerance to ignore the latter command. Default
  //   * value is 0.001. (.1% duty cycle)
  //   *
  //   * @param tolerance The new power tolerance.
  //   */
  //  public void setTolerance(double tolerance) {
  //    motorInternal.setCacheTolerance(tolerance);
  //  }

  private double currentLimitAmps = -1;

  /**
   * Sets the current limit of the motor.
   *
   * <p>Motors can draw hell of a lot of current when pushed hard, and if too many motors draw too
   * much current, the voltage sag can cause things to brown out. To prevent this, the duty cycle of
   * the motor is reduced when the limit is hit.
   *
   * <p>This applies a supply current limit. The better way to do current limiting is stator
   * limiting where the current is measured from the motor stators, but FTC hardware doesn't have
   * that. You can approximate it by taking the supply current draw and dividing it by duty cycle,
   * but it's less accurate and it's more costly on performance.
   */
  public void setCurrentLimit(double currentLimitAmps) {
    this.currentLimitAmps = currentLimitAmps;
  }

  private final LinearFilter currentFilter = LinearFilter.movingAverage(3);

  /**
   * Sets the duty cycle of the motor.
   *
   * @param dutyCycle The duty cycle to set. Clamped between -1 and 1.
   */
  public void set(double dutyCycle) {
    dutyCycle = MathUtil.clamp(dutyCycle, -1, 1);
    //    if (currentLimitAmps > 0 && currentFilter.calculate(getCurrentDraw()) > currentLimitAmps)
    // {
    //      dutyCycle *= currentLimitAmps / currentFilter.lastValue();
    //    }
    motorInternal.setPower(dutyCycle);
  }

  /**
   * Set the voltage of the motor.
   *
   * <p>This compensates for voltage sag by raising the duty cycle when voltage drops, allowing for
   * a more consistent experience with commanding motor power.
   *
   * @param volts The motor voltage to set. From -12 to 12.
   */
  public void setVoltage(double volts) {
    var currentVoltage = MathUtil.clamp(-12, 12, voltageSensor.getVoltage());
    set(volts / currentVoltage);
  }

  //
  //  /**
  //   * Gets the current draw of the motor.
  //   *
  //   * @return Current draw. Amps.
  //   */
  //  public double getCurrentDraw() {
  //    return currentSensor.get();
  //  }

  private double conversionFactor = 1;

  /**
   * Sets the motor's conversion factor.
   *
   * <p>Values > 1 are reductions, so a 4:1 reduction would make a factor of 4.
   *
   * <p>A common use case is to convert from encoder ticks to rotations.
   *
   * @param conversionFactor The new conversion factor.
   */
  public void setConversionFactor(double conversionFactor) {
    this.conversionFactor = conversionFactor;
  }

  private double offset = 0;

  /**
   * Gets the motor's current position.
   *
   * @return The position. Units are in encoder ticks by default, but can be changed with {@link
   *     Motor#setConversionFactor(double conversionFactor)}.
   */
  public double getPosition() {
    return motorInternal.getCurrentPosition() / conversionFactor - offset;
  }

  public void setPosition(double position) {
    offset += getPosition() - position;
  }

  private double lastPos = 0;
  private double lastTime = -1;

  /**
   * Gets the motor's current velocity.
   *
   * <p>We calculate motor velocity ourselves because REV sucks and only calculates velocity at 20
   * hz.
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
    var currentPos = getPosition();
    var currentTime = Utils.getTimeSeconds();
    var velocity = (currentPos - lastPos) / (currentTime - lastTime);
    lastPos = currentPos;
    lastTime = currentTime;
    return velocity;
  }
}
