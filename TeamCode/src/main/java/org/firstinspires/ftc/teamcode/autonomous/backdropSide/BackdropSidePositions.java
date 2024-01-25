package org.firstinspires.ftc.teamcode.autonomous.backdropSide;

import com.acmerobotics.roadrunner.Pose2d;

/// !!! BLUE
public class BackdropSidePositions {

    /// blue spike

    public static Pose2d[] blueSpikePos = {
            new Pose2d(30, 27, Math.toRadians(- 90)),
            new Pose2d(38, 17, Math.toRadians(- 90)),
            new Pose2d(28, 6, Math.toRadians(- 90))
    };

    /// blue backdrop
    static double blueBackdropY = 40;
    public static Pose2d[] blueBackdropPos = {
            new Pose2d(23, blueBackdropY, Math.toRadians(- 90)),
            new Pose2d(31, blueBackdropY, Math.toRadians(- 90)),
            new Pose2d(36, blueBackdropY, Math.toRadians(- 90))
    };

    public static Pose2d blueIntermediarPos = new Pose2d(52, 30, Math.toRadians(-83));
    public static Pose2d blueStackPos = new Pose2d(52, -70, Math.toRadians(-83));
    public static Pose2d blueParkInsidePos = new Pose2d(45, 35, Math.toRadians(- 90));
    public static Pose2d blueParkOutsidePos = new Pose2d(3, 38, Math.toRadians(- 90));


    public static Pose2d[] redSpikePos = {
            new Pose2d(29, -3, Math.toRadians(90)),
            new Pose2d(38, -12, Math.toRadians(90)),
            new Pose2d(30, -22, Math.toRadians(90))
    };

    public static Pose2d[] redBackdropPos = {
            new Pose2d(37, -40, Math.toRadians(90)),
            new Pose2d(29, -40, Math.toRadians(90)),
            new Pose2d(21, -40, Math.toRadians(90))
    };

    public static Pose2d redParkInsidePos = new Pose2d(45, 35, Math.toRadians(90));
    public static Pose2d redParkOutsidePos = new Pose2d(3, -38, Math.toRadians(90));


}
