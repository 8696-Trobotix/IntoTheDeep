// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogAbsoluteEncoder extends AbsoluteEncoder {
  private final AnalogInput sensor;

  public AnalogAbsoluteEncoder(OpMode opMode, String name) {
    super(0, 360);

    sensor = opMode.hardwareMap.analogInput.get(name);
  }

  @Override
  double getRawPosition() {
    return sensor.getVoltage() / sensor.getMaxVoltage();
  }
}
