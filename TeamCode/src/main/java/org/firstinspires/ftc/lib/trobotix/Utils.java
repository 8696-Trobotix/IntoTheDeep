// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

public class Utils {
  private Utils() {}

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
}
