// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Wrapper for {@link PhotonAdvancedDcMotor} for extra functionality and cleaner code. */
public class Motor {
  private final PhotonAdvancedDcMotor motorInternal;
  private final PhotonLynxVoltageSensor voltageSensor;

  public Motor(OpMode opMode, String name) {
    motorInternal = new PhotonAdvancedDcMotor((PhotonDcMotor) opMode.hardwareMap.dcMotor.get(name));
    voltageSensor = opMode.hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
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
    motorInternal
        .getMotor()
        .setDirection(inverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
  }

  /**
   * Typically, we want the motor to stop when we stop applying power to it. However, we
   * occasionally want it to keep coasting when no more power is applied.
   *
   * @param brake Whether or not the motor brakes when no power is applied.
   */
  public void setIdleBrake(boolean brake) {
    motorInternal
        .getMotor()
        .setZeroPowerBehavior(
            brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
  }

  /**
   * Sets the tolerance for which setting new power values is ignored.
   *
   * <p>It's a waste of processor time to tell the motor to move at .9 power and immediately tell it
   * to move at .901 power, so we use the cache tolerance to ignore the latter command. Default
   * value is 0.001. (.1% duty cycle)
   *
   * @param tolerance The new power tolerance.
   */
  public void setTolerance(double tolerance) {
    motorInternal.setCacheTolerance(tolerance);
  }

  /**
   * Sets the duty cycle of the motor.
   *
   * @param dutyCycle The duty cycle to set. From -1 to 1.
   */
  public void set(double dutyCycle) {
    motorInternal.setPower(MathUtil.clamp(dutyCycle, -1, 1));
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
    set(volts / voltageSensor.getCachedVoltage());
  }

  /**
   * Gets the current draw of the motor.
   *
   * @return Current draw. Amps.
   */
  public double getCurrentDraw() {
    return motorInternal.getMotor().getCorrectedCurrent(CurrentUnit.AMPS);
  }

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

  /**
   * Gets the motor's current position.
   *
   * @return The position. Units are in encoder ticks by default, but can be changed with {@link
   *     Motor#setConversionFactor(double conversionFactor)}.
   */
  public double getPosition() {
    return motorInternal.getMotor().getCurrentPosition() / conversionFactor;
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
