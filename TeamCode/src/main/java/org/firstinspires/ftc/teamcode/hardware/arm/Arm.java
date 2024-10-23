// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware.arm;

import static org.firstinspires.ftc.teamcode.hardware.arm.ArmControlThread.maxSpeedRadPerSec;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.lib.wpilib.Timer;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;

public class Arm extends SubsystemBase {
  private final ArmControlThread controlThread;

  public Arm(OpMode opMode) {
    controlThread = new ArmControlThread(opMode);
  }

  private void runVel(double velRadPerSec) {
    controlThread.setTargetVel(velRadPerSec);
  }

  public Command maintainAngle() {
    return run(() -> runVel(0));
  }

  public Command voltageSweep(double rate) {
    final var timer = new Timer();
    return runOnce(timer::restart)
        .andThen(
            run(
                () -> {
                  controlThread.setVoltage(timer.get() * rate);
                }))
        .until(() -> timer.hasElapsed(rate * 12));
  }

  public Command raise() {
    return run(() -> runVel(maxSpeedRadPerSec));
  }

  public Command lower() {
    return run(() -> runVel(-maxSpeedRadPerSec));
  }
}
