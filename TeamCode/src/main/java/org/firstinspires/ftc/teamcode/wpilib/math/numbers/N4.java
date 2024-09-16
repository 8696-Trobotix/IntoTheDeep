package org.firstinspires.ftc.teamcode.wpilib.math.numbers;

import org.firstinspires.ftc.teamcode.wpilib.math.Num;
import org.firstinspires.ftc.teamcode.wpilib.math.Nat;

/** A class representing the number 0. */
public final class N4 extends Num implements Nat<N4> {
    private N4() {}

    /**
     * The integer this class represents.
     *
     * @return The literal number 4.
     */
    @Override
    public int getNum() {
        return 4;
    }

    /** The singleton instance of this class. */
    public static final N4 instance = new N4();
}