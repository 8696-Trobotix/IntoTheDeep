// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.teamutils;

public class Utils {
  private Utils() {}

  public static double getTimeSeconds() {
    return System.nanoTime() / 1e9;
  }

  public static double average(double... numbers) {
    double sum = 0;
    for (double number : numbers) {
      sum += number;
    }
    return sum /= numbers.length;
  }

  public static double minimum(double... numbers) {
    double min = Double.POSITIVE_INFINITY;
    for (double number : numbers) {
      min = Math.min(min, number);
    }
    return min;
  }
}
