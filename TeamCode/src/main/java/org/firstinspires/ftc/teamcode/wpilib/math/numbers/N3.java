package org.firstinspires.ftc.teamcode.wpilib.math.numbers;

import org.firstinspires.ftc.teamcode.wpilib.math.Num;
import org.firstinspires.ftc.teamcode.wpilib.math.Nat;

/** A class representing the number 0. */
public final class N3 extends Num implements Nat<N3> {
    private N3() {}

    /**
     * The integer this class represents.
     *
     * @return The literal number 3.
     */
    @Override
    public int getNum() {
        return 3;
    }

    /** The singleton instance of this class. */
    public static final N3 instance = new N3();
}
