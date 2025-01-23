// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Wrapper for {@link DcMotorEx} for extra functionality and cleaner code. */
public class Motor {
  private final DcMotorEx motorInternal;
  private final DoubleSupplier voltageSupplier;

  private final BaseOpMode.CachedValue.CachedPosition cachedPosition;

  /**
   * Constructs a new Motor without an encoder, using the specified op mode and name.
   *
   * @param opMode The current op mode.
   * @param name The name of the motor in the configuration.
   */
  public Motor(BaseOpMode opMode, String name) {
    this(opMode, name, false);
  }

  /**
   * Constructs a new Motor using the specified op mode and name.
   *
   * @param opMode The current op mode.
   * @param name The name of the motor in the configuration.
   * @param hasEncoder If the motor has an encoder or not.
   */
  public Motor(BaseOpMode opMode, String name, boolean hasEncoder) {
    motorInternal = (DcMotorEx) opMode.hardwareMap.dcMotor.get(name);
    voltageSupplier = opMode.busVoltageSupplier();

    motorInternal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorInternal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motorInternal.setDirection(DcMotorSimple.Direction.FORWARD);

    cachedPosition =
        new BaseOpMode.CachedValue.CachedPosition(
            hasEncoder
                ? () ->
                    (inverted
                                ? -motorInternal.getCurrentPosition()
                                : motorInternal.getCurrentPosition())
                            / conversionFactor
                        - offset
                : () -> 0);
  }

  private boolean inverted = false;

  /**
   * By default, a motor will typically move counterclockwise when positive voltage is sent. (The
   * only exception is Neverest motors which is clockwise positive.)
   *
   * <p>However, that isn't always desired. Sometimes we may want to have the opposite behaviour.
   *
   * <p>Note: Setting this after the motor has already moved causes undefined behavior for {@link
   * Motor#getPosition()}. Make sure to call {@link Motor#setPosition(double)} if that is done.
   *
   * @param inverted If the motor is inverted or not.
   */
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
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

  private double lastDutyCycle = 0;
  private double tolerance = .001;

  /**
   * Set the voltage of the motor.
   *
   * <p>This compensates for voltage sag by raising the duty cycle when voltage drops, allowing for
   * a more consistent experience with commanding motor power.
   *
   * @param volts The motor voltage to set. From -12 to 12.
   */
  public void set(double volts) {
    // Clamp voltage
    var busVoltage = MathUtil.clamp(0, 12, voltageSupplier.getAsDouble());
    volts = MathUtil.clamp(volts, -busVoltage, busVoltage);

    var dutyCycle = volts / busVoltage;
    if (inverted) {
      dutyCycle *= -1;
    }
    if (tolerance > 0 && !MathUtil.isNear(lastDutyCycle, dutyCycle, tolerance)) {
      motorInternal.setPower(dutyCycle);
      lastDutyCycle = dutyCycle;
    }
  }

  /**
   * Gets the current draw of the motor.
   *
   * @return Current draw. Amps.
   */
  public double getCurrentDraw() {
    return motorInternal.getCurrent(CurrentUnit.AMPS);
  }

  private double conversionFactor = 1;

  /**
   * Sets the motor's conversion factor.
   *
   * <p>Values > 1 are reductions, so a 4:1 reduction would make a factor of 4.
   *
   * <p>A common use case is to convert from encoder ticks to rotations.
   *
   * <p>Note: Setting this after the motor has already moved causes undefined behavior for {@link
   * Motor#getPosition()}.
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
   * <p>This always returns 0 if there's no encoder connected.
   *
   * @return The position.
   */
  public double getPosition() {
    return cachedPosition.latestPosition;
  }

  /**
   * Sets the motor's current position.
   *
   * <p>Does nothing if there's no encoder connected.
   *
   * @param position The new position.
   */
  public void setPosition(double position) {
    offset += cachedPosition.latestPosition - position;
    cachedPosition.latestPosition = position;
  }

  /**
   * Gets the motor's current velocity.
   *
   * <p>We calculate motor velocity ourselves because REV sucks and only calculates velocity at 20
   * hz.
   *
   * <p>This always returns 0 if there's no encoder connected.
   *
   * @return The velocity. Numerator units follow {@link Motor#getPosition()}. Denominator unit is
   *     seconds.
   */
  public double getVelocity() {
    return cachedPosition.latestVelocity;
  }
}
