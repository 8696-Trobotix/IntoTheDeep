// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import static org.firstinspires.ftc.lib.wpilib.commands.Commands.none;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;

public class Claw extends SubsystemBase {
  //  private final Servo servo;

  public Claw(OpMode opMode) {
    //    servo = opMode.hardwareMap.servo.get("servo1");
  }

  public Command open() {
    //    return run(() -> servo.setPosition(1));
    return none();
  }

  public Command close() {
    //    return run(() -> servo.setPosition(0));
    return none();
  }
}
