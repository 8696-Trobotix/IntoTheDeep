// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.wpilib.Timer;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;

public class Claw extends SubsystemBase {
  private final Servo servo;

  public Claw(OpMode opMode) {
    servo = opMode.hardwareMap.servo.get("armServo");
    servo.scaleRange(0, 1);
  }

  @Override
  public void periodic() {
    Telemetry.addData("Claw/Position", servo.getPosition());
  }

  public Command open() {
    return run(
        () -> {
          Telemetry.addData("Claw/Commanded", 1);
          servo.setPosition(1);
        });
  }

  public Command close() {
    return run(
        () -> {
          Telemetry.addData("Claw/Commanded", 0);
          servo.setPosition(0);
        });
  }

  public Command servoSweep(double rate) {
    final Timer timer = new Timer();
    return runOnce(timer::restart)
        .andThen(
            run(
                () -> {
                  var pos = timer.get() * rate;
                  Telemetry.addData("Claw/Commanded", pos);
                  servo.setPosition(pos);
                }))
        .until(() -> timer.hasElapsed(1.0 / rate))
        .finallyDo(timer::stop);
  }
}
