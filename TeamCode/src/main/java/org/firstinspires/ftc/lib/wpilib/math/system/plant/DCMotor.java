// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.system.plant;

import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

/** Holds the constants for a DC motor. */
public class DCMotor {
  /** Voltage at which the motor constants were measured. */
  public final double nominalVoltageVolts;

  /** Torque when stalled. */
  public final double stallTorqueNewtonMeters;

  /** Current draw when stalled. */
  public final double stallCurrentAmps;

  /** Current draw under no load. */
  public final double freeCurrentAmps;

  /** Angular velocity under no load. */
  public final double freeSpeedRadPerSec;

  /** Motor internal resistance. */
  public final double rOhms;

  /** Motor velocity constant. */
  public final double KvRadPerSecPerVolt;

  /** Motor torque constant. */
  public final double KtNMPerAmp;

  /**
   * Constructs a DC motor.
   *
   * @param nominalVoltageVolts Voltage at which the motor constants were measured.
   * @param stallTorqueNewtonMeters Torque when stalled.
   * @param stallCurrentAmps Current draw when stalled.
   * @param freeCurrentAmps Current draw under no load.
   * @param freeSpeedRadPerSec Angular velocity under no load.
   * @param numMotors Number of motors in a gearbox.
   */
  public DCMotor(
      double nominalVoltageVolts,
      double stallTorqueNewtonMeters,
      double stallCurrentAmps,
      double freeCurrentAmps,
      double freeSpeedRadPerSec,
      int numMotors) {
    this.nominalVoltageVolts = nominalVoltageVolts;
    this.stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors;
    this.stallCurrentAmps = stallCurrentAmps * numMotors;
    this.freeCurrentAmps = freeCurrentAmps * numMotors;
    this.freeSpeedRadPerSec = freeSpeedRadPerSec;

    this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
    this.KvRadPerSecPerVolt =
        freeSpeedRadPerSec / (nominalVoltageVolts - rOhms * this.freeCurrentAmps);
    this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;
  }

  /**
   * Calculate current drawn by motor with given speed and input voltage.
   *
   * @param speedRadiansPerSec The current angular velocity of the motor.
   * @param voltageInputVolts The voltage being applied to the motor.
   * @return The estimated current.
   */
  public double getCurrent(double speedRadiansPerSec, double voltageInputVolts) {
    return -1.0 / KvRadPerSecPerVolt / rOhms * speedRadiansPerSec + 1.0 / rOhms * voltageInputVolts;
  }

  /**
   * Calculate current drawn by motor for a given torque.
   *
   * @param torqueNm The torque produced by the motor.
   * @return The current drawn by the motor.
   */
  public double getCurrent(double torqueNm) {
    return torqueNm / KtNMPerAmp;
  }

  /**
   * Calculate torque produced by the motor with a given current.
   *
   * @param currentAmpere The current drawn by the motor.
   * @return The torque output.
   */
  public double getTorque(double currentAmpere) {
    return currentAmpere * KtNMPerAmp;
  }

  /**
   * Calculate the voltage provided to the motor for a given torque and angular velocity.
   *
   * @param torqueNm The torque produced by the motor.
   * @param speedRadiansPerSec The current angular velocity of the motor.
   * @return The voltage of the motor.
   */
  public double getVoltage(double torqueNm, double speedRadiansPerSec) {
    return 1.0 / KvRadPerSecPerVolt * speedRadiansPerSec + 1.0 / KtNMPerAmp * rOhms * torqueNm;
  }

  /**
   * Calculates the angular speed produced by the motor at a given torque and input voltage.
   *
   * @param torqueNm The torque produced by the motor.
   * @param voltageInputVolts The voltage applied to the motor.
   * @return The angular speed of the motor.
   */
  public double getSpeed(double torqueNm, double voltageInputVolts) {
    return voltageInputVolts * KvRadPerSecPerVolt
        - 1.0 / KtNMPerAmp * torqueNm * rOhms * KvRadPerSecPerVolt;
  }

  /**
   * Returns a copy of this motor with the given gearbox reduction applied.
   *
   * @param gearboxReduction The gearbox reduction.
   * @return A motor with the gearbox reduction applied.
   */
  public DCMotor withReduction(double gearboxReduction) {
    return new DCMotor(
        nominalVoltageVolts,
        stallTorqueNewtonMeters * gearboxReduction,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRadPerSec / gearboxReduction,
        1);
  }

  /**
   * Gets the 6000 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0001(int numMotors) {
    return new DCMotor(
        12, 0.144157755, 11, .3, Units.rotationsPerMinuteToRadiansPerSecond(6000), numMotors);
  }

  /**
   * Gets the 1620 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0003(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(3.7);
  }

  /**
   * Gets the 1150 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0005(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(5.2);
  }

  /**
   * Gets the 435 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0014(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(13.7);
  }

  /**
   * Gets the 312 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0019(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(19.2);
  }

  /**
   * Gets the 223 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0027(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(26.9);
  }

  /**
   * Gets the 117 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0051(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(50.9);
  }

  /**
   * Gets the 84 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0071(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(71.2);
  }

  /**
   * Gets the 60 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0100(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(99.5);
  }

  /**
   * Gets the 43 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0139(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(139);
  }

  /**
   * Gets the 30 RPM version of the GoBILDA 5203 Series Yellow Jacket motor.
   *
   * @param numMotors The number of motors on the mechanism.
   */
  public static DCMotor getGoBILDA5203_0188(int numMotors) {
    return getGoBILDA5203_0001(numMotors).withReduction(188);
  }
}
