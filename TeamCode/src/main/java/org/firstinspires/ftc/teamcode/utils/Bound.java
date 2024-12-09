package org.firstinspires.ftc.teamcode.utils;

public class Bound {

    public static double calculate(double val, double min, double max) {
        val = Math.max(min, val);
        val = Math.min(max, val);

        return val;
    }

}
