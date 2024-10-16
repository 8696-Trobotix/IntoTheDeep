// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import java.util.ArrayList;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.utils.Units;

public class Utils {
  private Utils() {}

  public static boolean IS_ON_RED = false;

  public static double getTimeSeconds() {
    return System.nanoTime() / 1e9;
  }

  public static double average(double... numbers) {
    var sum = 0.0;
    for (var number : numbers) {
      sum += number;
    }
    return sum / numbers.length;
  }

  public static double minimum(double... numbers) {
    var min = Double.POSITIVE_INFINITY;
    for (var number : numbers) {
      min = Math.min(min, number);
    }
    return min;
  }

  public static double maximum(double... numbers) {
    var max = Double.NEGATIVE_INFINITY;
    for (var number : numbers) {
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

  private static final ArrayList<EndableThread> threads = new ArrayList<>();

  protected static void registerThread(EndableThread thread) {
    threads.add(thread);
  }

  protected static void startThreads() {
    threads.forEach(
        (thread) -> {
          try {
            thread.start();
          } catch (Exception e) {
            throw new RuntimeException(
                "Failed to start " + thread.NAME + " with exception: " + e);
          }
        });
  }

  protected static void endThreads() {
    threads.forEach(EndableThread::disable);
    threads.clear();
  }
}
