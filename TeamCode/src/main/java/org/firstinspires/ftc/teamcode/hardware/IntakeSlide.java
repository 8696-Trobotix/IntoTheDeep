// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.hardware;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.commands.Command;
import org.firstinspires.ftc.lib.wpilib.commands.SubsystemBase;

public class IntakeSlide extends SubsystemBase {
  private final Motor motor;

  public IntakeSlide(BaseOpMode opMode) {
    motor = new Motor(opMode, "intakeSlide");
  }

  public Command manualControl(DoubleSupplier control) {
    return run(() -> motor.set(control.getAsDouble() * 12));
  }
}
