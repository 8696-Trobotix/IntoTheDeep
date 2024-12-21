// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.hardware.slide.Slide;

@TeleOp
public class CharacterizeSlide extends BaseOpMode {
  @Override
  protected void startup() {
    var slide = new Slide(this);

    slide.setDefaultCommand(
        slide.runVoltage(
            () -> {
              var control = -primaryController().getLeftX();
              control = 2 * Math.copySign(control * control, control);
              telemetry.addData("Voltage", control);
              return control;
            }));
  }
}
