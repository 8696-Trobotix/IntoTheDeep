// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.interpolation;

/**
 * An object should extend interpolatable if you wish to interpolate between a lower and upper
 * bound, such as a robot position on the field between timesteps. This behavior can be linear or
 * nonlinear.
 *
 * @param <T> The class that is interpolatable.
 */
public interface Interpolatable<T> {
  /**
   * Return the interpolated value. This object is assumed to be the starting position, or lower
   * bound.
   *
   * @param endValue The upper bound, or end.
   * @param t How far between the lower and upper bound we are. This should be bounded in [0, 1].
   * @return The interpolated value.
   */
  T interpolate(T endValue, double t);
}
