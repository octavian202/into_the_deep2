package org.firstinspires.ftc.teamcode.utils;

public class Map {

    public static double calculate(double val, double min1, double max1, double min2, double max2) {
        double percentage = (val - min1) / (max1 - min1);
        return (percentage) * (max2 - min2) + min2;
    }

}
