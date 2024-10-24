// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;

public class Claw extends SubsystemBase {
  private final PhotonServo servo;

  public Claw(OpMode opMode) {
    servo = (PhotonServo) opMode.hardwareMap.get("armServo");
  }

  @Override
  public void periodic() {
    Telemetry.addData("Claw/Position", servo.getPosition());
  }

  public Command open() {
    return run(() -> {
      Telemetry.addData("Claw/Commanded", 1);
      servo.setPosition(1);
    });
  }

  public Command close() {
    return run(() -> {
      Telemetry.addData("Claw/Commanded", 0);
      servo.setPosition(0);
    });
  }
}
