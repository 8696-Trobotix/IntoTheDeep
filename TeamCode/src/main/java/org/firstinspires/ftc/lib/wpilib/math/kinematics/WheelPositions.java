// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.interpolation.Interpolatable;

/**
 * Interface for wheel positions.
 *
 * @param <T> Wheel positions type.
 */
public interface WheelPositions<T extends WheelPositions<T>> extends Interpolatable<T> {
  /**
   * Returns a copy of this instance.
   *
   * @return A copy.
   */
  T copy();
}
