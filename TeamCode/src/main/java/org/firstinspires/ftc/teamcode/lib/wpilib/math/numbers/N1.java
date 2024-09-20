// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.lib.wpilib.math.numbers;

import org.firstinspires.ftc.teamcode.lib.wpilib.math.Nat;
import org.firstinspires.ftc.teamcode.lib.wpilib.math.Num;

/** A class representing the number 0. */
public final class N1 extends Num implements Nat<N1> {
  private N1() {}

  /**
   * The integer this class represents.
   *
   * @return The literal number 1.
   */
  @Override
  public int getNum() {
    return 1;
  }

  /** The singleton instance of this class. */
  public static final N1 instance = new N1();
}
