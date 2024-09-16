package org.firstinspires.ftc.teamcode.wpilib.math;

import org.firstinspires.ftc.teamcode.wpilib.math.numbers.*;

/**
 * A natural number expressed as a java class.
 * The counterpart to {@link Num} that should be used as a concrete value.
 *
 * @param <T> The {@link Num} this represents.
 */
public interface Nat<T extends Num> {
    /**
     * The number this interface represents.
     *
     * @return The number backing this value.
     */
    int getNum();

    /**
     * Returns the Nat instance for 0.
     *
     * @return The Nat instance for 0.
     */
    static Nat<N0> N0() {
        return N0.instance;
    }

    /**
     * Returns the Nat instance for 1.
     *
     * @return The Nat instance for 1.
     */
    static Nat<N1> N1() {
        return N1.instance;
    }

    /**
     * Returns the Nat instance for 2.
     *
     * @return The Nat instance for 2.
     */
    static Nat<N2> N2() {
        return N2.instance;
    }

    /**
     * Returns the Nat instance for 3.
     *
     * @return The Nat instance for 3.
     */
    static Nat<N3> N3() {
        return N3.instance;
    }

    /**
     * Returns the Nat instance for 4.
     *
     * @return The Nat instance for 4.
     */
    static Nat<N4> N4() {
        return N4.instance;
    }

    /**
     * Returns the Nat instance for 5.
     *
     * @return The Nat instance for 5.
     */
    static Nat<N5> N5() {
        return N5.instance;
    }

    // Couldn't be bothered to go past N5
}
