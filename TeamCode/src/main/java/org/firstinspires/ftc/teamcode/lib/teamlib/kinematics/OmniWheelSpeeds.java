package org.firstinspires.ftc.teamcode.lib.teamlib.kinematics;

import org.firstinspires.ftc.teamcode.lib.teamlib.Utils;

/** Represents the wheel speeds for a omni wheel drivetrain. */
public class OmniWheelSpeeds {
    public double[] speeds;

    public OmniWheelSpeeds(double... wheelSpeeds) {
        this.speeds = wheelSpeeds;
    }

    public OmniWheelSpeeds() {
        this(0, 0, 0);
    }

    public OmniWheelSpeeds plus(OmniWheelSpeeds wheelPositions) {
        double[] newPositions = new double[this.speeds.length];
        for (int i = 0; i < this.speeds.length; i++) {
            newPositions[i] = this.speeds[i] + wheelPositions.speeds[i];
        }
        return new OmniWheelSpeeds(newPositions);
    }

    public OmniWheelSpeeds minus(OmniWheelSpeeds wheelPositions) {
        return plus(wheelPositions.unaryMinus());
    }

    public OmniWheelSpeeds unaryMinus() {
        double[] newPositions = new double[this.speeds.length];
        for (int i = 0; i < this.speeds.length; i++) {
            newPositions[i] = -this.speeds[i];
        }
        return new OmniWheelSpeeds(newPositions);
    }

    public OmniWheelSpeeds times(double scalar) {
        double[] newPositions = new double[this.speeds.length];
        for (int i = 0; i < this.speeds.length; i++) {
            newPositions[i] = this.speeds[i] * scalar;
        }
        return new OmniWheelSpeeds(newPositions);
    }

    public OmniWheelSpeeds divide(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more wheels may be
     * above the max attainable speed for the driving motor on that wheel. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested wheel speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between wheels.
     *
     * @param wheelSpeeds The speeds to renormalize.
     * @param attainableMaxSpeedMetersPerSec The max speed that can be attained by the wheels.
     * @return If renormalizing is not needed, wheelSpeeds. If it is needed, a new OmniWheelSpeeds
     *     with the new velocities.
     */
    public static OmniWheelSpeeds desaturateWheelSpeeds(
            OmniWheelSpeeds wheelSpeeds, double attainableMaxSpeedMetersPerSec) {
        double highestSpeed = Utils.maximum(wheelSpeeds.speeds);
        if (highestSpeed > attainableMaxSpeedMetersPerSec) {
            return wheelSpeeds.times(attainableMaxSpeedMetersPerSec / highestSpeed);
        }
        return wheelSpeeds;
    }
}
