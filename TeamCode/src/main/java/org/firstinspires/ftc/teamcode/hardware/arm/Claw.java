// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.DoubleSupplier;
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
    return run(() -> setPos(0));
  }

  public Command close() {
    return run(() -> setPos(1));
  }

  public Command setPos(DoubleSupplier posSupplier) {
    return run(() -> setPos(posSupplier.getAsDouble()));
  }

  private void setPos(double pos) {
    Telemetry.addData("Claw/Commanded", pos);
    servo.setPosition(pos);
  }

  public Command servoSweep(double rate) {
    final Timer timer = new Timer();
    return runOnce(timer::restart)
        .andThen(
            run(
                () -> setPos(timer.get() * rate)))
        .until(() -> timer.hasElapsed(1.0 / rate))
        .finallyDo(timer::stop);
  }
}
