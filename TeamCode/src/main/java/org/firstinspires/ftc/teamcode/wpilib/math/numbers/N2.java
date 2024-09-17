// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.wpilib.math.numbers;

import org.firstinspires.ftc.teamcode.wpilib.math.Nat;
import org.firstinspires.ftc.teamcode.wpilib.math.Num;

/** A class representing the number 0. */
public final class N2 extends Num implements Nat<N2> {
  private N2() {}

  /**
   * The integer this class represents.
   *
   * @return The literal number 2.
   */
  @Override
  public int getNum() {
    return 2;
  }

  /** The singleton instance of this class. */
  public static final N2 instance = new N2();
}
