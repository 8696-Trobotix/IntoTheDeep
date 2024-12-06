// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RelativeEncoder {
  private final DcMotorEx motor;
  private final double conversionFactor;

  public RelativeEncoder(OpMode opMode, String name, double conversionFactor) {
    motor = (DcMotorEx) opMode.hardwareMap.dcMotor.get(name);
    this.conversionFactor = conversionFactor;

    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  private double offset = 0;

  public void setPosition(double position) {
    offset += getPosition() - position;
  }

  private boolean inverted = false;

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  public double getPosition() {
    return (inverted ? -motor.getCurrentPosition() : motor.getCurrentPosition()) / conversionFactor
        - offset;
  }
}
