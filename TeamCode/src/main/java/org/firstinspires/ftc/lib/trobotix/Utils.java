// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import java.util.ArrayList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class Utils {
  private Utils() {}

  public static boolean IS_ON_RED = false;

  public static ReadWriteLock THREAD_LOCK = new ReentrantReadWriteLock();

  public static double getTimeSeconds() {
    return System.nanoTime() / 1e9;
  }

  public static double average(double... numbers) {
    double sum = 0;
    for (double number : numbers) {
      sum += number;
    }
    return sum / numbers.length;
  }

  public static double minimum(double... numbers) {
    double min = Double.POSITIVE_INFINITY;
    for (double number : numbers) {
      min = Math.min(min, number);
    }
    return min;
  }

  public static double maximum(double... numbers) {
    double max = Double.NEGATIVE_INFINITY;
    for (double number : numbers) {
      max = Math.max(max, number);
    }
    return max;
  }

  public static final double FIELD_SIZE = Units.feetToMeters(12);

  public static Pose2d flipAllianceOnRed(Pose2d pose) {
    if (IS_ON_RED) {
      return new Pose2d(
          flipAllianceOnRed(pose.getTranslation()), flipAllianceOnRed(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static Translation2d flipAllianceOnRed(Translation2d translation) {
    if (IS_ON_RED) {
      return new Translation2d(FIELD_SIZE - translation.getX(), FIELD_SIZE - translation.getY());
    } else {
      return translation;
    }
  }

  public static Rotation2d flipAllianceOnRed(Rotation2d rotation) {
    if (IS_ON_RED) {
      return rotation.plus(Rotation2d.fromDegrees(180));
    } else {
      return rotation;
    }
  }
}
