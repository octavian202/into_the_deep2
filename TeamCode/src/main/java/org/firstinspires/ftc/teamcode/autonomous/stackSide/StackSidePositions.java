package org.firstinspires.ftc.teamcode.autonomous.stackSide;

import com.acmerobotics.roadrunner.Pose2d;

public class StackSidePositions {

    public static Pose2d[] blueSpikePos = {
            new Pose2d(25, 2, Math.toRadians(45)),
            new Pose2d(25, 0, Math.toRadians(0)),
            new Pose2d(21, -10, Math.toRadians(0))
    };

    static double blueBackdropY = 89;
    public static Pose2d[] blueBackdropPos = {
            new Pose2d(21d, blueBackdropY, Math.toRadians(-90)),
            new Pose2d(30d, blueBackdropY, Math.toRadians(-90)),
            new Pose2d(36d, blueBackdropY, Math.toRadians(-90))
    };

    public static Pose2d blueParkPos = new Pose2d(50, 88, Math.toRadians(-90));

    public static Pose2d[] redSpikePos = {
            new Pose2d(21d, 10, Math.toRadians(0)),
            new Pose2d(27d, 0, Math.toRadians(0)),
            new Pose2d(25d, -2, Math.toRadians(-45))
    };

    static double redBackdropY = -88;
    public static Pose2d[] redBackdropPos = {
            new Pose2d(37d, redBackdropY, Math.toRadians(90)),
            new Pose2d(29d, redBackdropY, Math.toRadians(90)),
            new Pose2d(20d, redBackdropY, Math.toRadians(90))
    };
    public static Pose2d redParkPos = new Pose2d(50, -82, Math.toRadians(90));

}
