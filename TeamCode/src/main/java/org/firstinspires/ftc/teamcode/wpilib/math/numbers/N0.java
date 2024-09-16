package org.firstinspires.ftc.teamcode.wpilib.math.numbers;

import org.firstinspires.ftc.teamcode.wpilib.math.Num;
import org.firstinspires.ftc.teamcode.wpilib.math.Nat;

/** A class representing the number 0. */
public final class N0 extends Num implements Nat<N0> {
    private N0() {}

    /**
     * The integer this class represents.
     *
     * @return The literal number 0.
     */
    @Override
    public int getNum() {
        return 0;
    }

    /** The singleton instance of this class. */
    public static final N0 instance = new N0();
}