package org.firstinspires.ftc.teamcode.teamutils;

public class Utils {
    private Utils() {}

    public static double getTimeSeconds() {
        return System.nanoTime() / 1e9;
    }
}
