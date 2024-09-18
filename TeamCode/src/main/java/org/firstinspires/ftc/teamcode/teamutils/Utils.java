// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.teamutils;

public class Utils {
  private Utils() {}

  public static double getTimeSeconds() {
    return System.nanoTime() / 1e9;
  }
}
