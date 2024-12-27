// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.controller;

import org.firstinspires.ftc.lib.wpilib.math.MathUtil;

public class ViperSlideFeedforward {
  public final double bottomPos, topPos, kS_bottom, kG_bottom, kS_top, kG_top, kV;

  public ViperSlideFeedforward(
      double bottomPos,
      double topPos,
      double kS_bottom,
      double kG_bottom,
      double kS_top,
      double kG_top,
      double kV) {
    this.bottomPos = bottomPos;
    this.topPos = topPos;
    this.kS_bottom = kS_bottom;
    this.kG_bottom = kG_bottom;
    this.kS_top = kS_top;
    this.kG_top = kG_top;
    this.kV = kV;
  }

  public double calculate(double currentPos, double velocity) {
    var constants = getConstants(currentPos);
    return constants[0] * Math.signum(velocity) + constants[1] + kV * velocity;
  }

  public double getMaxAchievableVel(double position) {
    var constants = getConstants(position);
    return (12 - constants[0] - constants[1]) / kV;
  }

  public double getMinAchievableVel(double position) {
    var constants = getConstants(position);
    return (-12 + constants[0] - constants[1]) / kV;
  }

  private double[] getConstants(double position) {
    double positionPercentage = MathUtil.inverseInterpolate(bottomPos, topPos, position);

    double kS = MathUtil.interpolate(kS_bottom, kS_top, positionPercentage);
    double kG = MathUtil.interpolate(kG_bottom, kG_top, positionPercentage);

    return new double[] {kS, kG};
  }
}
