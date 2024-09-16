package org.firstinspires.ftc.teamcode.wpilib.math.numbers;

import org.firstinspires.ftc.teamcode.wpilib.math.Num;
import org.firstinspires.ftc.teamcode.wpilib.math.Nat;

/** A class representing the number 0. */
public final class N5 extends Num implements Nat<N5> {
    private N5() {}

    /**
     * The integer this class represents.
     *
     * @return The literal number 5.
     */
    @Override
    public int getNum() {
        return 5;
    }

    /** The singleton instance of this class. */
    public static final N5 instance = new N5();
}