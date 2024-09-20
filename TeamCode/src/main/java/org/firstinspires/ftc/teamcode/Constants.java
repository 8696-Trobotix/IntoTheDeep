// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.utils.Units;

public class Constants {
  private Constants() {}

  public static final class Drivebase {
    public static final double DRIVE_MOTOR_MAX_RPM = 160;
    public static final double DRIVE_MOTOR_KT = 2.47154;
    public static final double DRIVE_MOTOR_ENCODER_TICKS_PER_ROTATION = 1120;

    public static final double TRACK_WIDTH = Units.inchesToMeters(18);
    public static final double TRACK_LENGTH = Units.inchesToMeters(18);

    public static final Translation2d[] WHEEL_POSITIONS =
        new Translation2d[] {
          new Translation2d(TRACK_WIDTH / 2, TRACK_LENGTH / 2),
          new Translation2d(TRACK_WIDTH / 2, -TRACK_LENGTH / 2),
          new Translation2d(-TRACK_WIDTH / 2, TRACK_LENGTH / 2),
          new Translation2d(-TRACK_WIDTH / 2, -TRACK_LENGTH / 2)
        };

    public static final double FRONT_LEFT_WHEEL_DIAMETER = Units.inchesToMeters(1);
    public static final double FRONT_RIGHT_WHEEL_DIAMETER = Units.inchesToMeters(1);
    public static final double BACK_LEFT_WHEEL_DIAMETER = Units.inchesToMeters(1);
    public static final double BACK_RIGHT_WHEEL_DIAMETER = Units.inchesToMeters(1);
  }
}
