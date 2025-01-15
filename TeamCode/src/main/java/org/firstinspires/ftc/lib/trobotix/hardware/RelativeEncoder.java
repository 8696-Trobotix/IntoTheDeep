// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;

public class RelativeEncoder {
  private final BaseOpMode.CachedValue.CachedPosition cachedPosition;

  public RelativeEncoder(OpMode opMode, String name, boolean inverted, double conversionFactor) {
    var motor = (DcMotorEx) opMode.hardwareMap.dcMotor.get(name);

    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    cachedPosition =
        new BaseOpMode.CachedValue.CachedPosition(
            () ->
                (inverted ? -motor.getCurrentPosition() : motor.getCurrentPosition())
                        / conversionFactor
                    - offset);
  }

  /** Gets the encoder's current position. */
  public double getPosition() {
    return cachedPosition.latestPosition;
  }

  private double offset = 0;

  public void setPosition(double position) {
    offset += cachedPosition.latestPosition - position;
    cachedPosition.latestPosition = position;
  }

  /**
   * Gets the encoder's current velocity.
   *
   * <p>We calculate motor velocity ourselves because REV sucks and only calculates velocity at 20
   * hz.
   */
  public double getVelocity() {
    return cachedPosition.latestVelocity;
  }
}
