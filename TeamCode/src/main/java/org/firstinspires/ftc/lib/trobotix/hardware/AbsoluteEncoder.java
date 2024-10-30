// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;

public abstract class AbsoluteEncoder {
  private final double minPosDeg;
  private final double maxPosDeg;

  AbsoluteEncoder(double minPosDeg, double maxPosDeg) {
    this.minPosDeg = minPosDeg;
    this.maxPosDeg = maxPosDeg;
  }

  public final Rotation2d getPosition() {
    return Rotation2d.fromDegrees(MathUtil.interpolate(minPosDeg, maxPosDeg, getRawPosition()));
  }

  /**
   * Gets the raw position of the encoder.
   *
   * <p>[0,1)
   */
  abstract double getRawPosition();
}
