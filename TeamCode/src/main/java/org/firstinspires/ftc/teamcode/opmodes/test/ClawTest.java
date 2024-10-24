// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTest extends LinearOpMode {

  @Override
  public void runOpMode() {
    Servo servo = this.hardwareMap.servo.get("armServo");
    waitForStart();
    while (opModeIsActive()) {
      servo.setPosition((gamepad1.left_stick_y + 1) / 2);
    }
  }
}
