package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class BasicIMU extends GyroEx {

    private BHI260IMU revIMU;

    /***
     * Heading relative to starting position
     */
    double globalHeading;

    /**
     * Heading relative to last offset
     */
    double relativeHeading;

    /**
     * Offset between global heading and relative heading
     */
    double offset;

    private int multiplier;

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     *
     * @param hw      Hardware map
     * @param imuName Name of sensor in configuration
     */
    public BasicIMU(HardwareMap hw, String imuName) {
        revIMU = hw.get(BHI260IMU.class, imuName);
        multiplier = 1;
    }

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub with the default configuration name of "imu"
     *
     * @param hw Hardware map
     */
    public BasicIMU(HardwareMap hw) {
        this(hw, "imu");
    }

    /**
     * Initializes gyro with default parameters.
     */
    public void init() {
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        init(parameters);
    }

    /**
     * Initializes gyro with custom parameters.
     */
    public void init(BHI260IMU.Parameters parameters) {
        revIMU.initialize(parameters);

        globalHeading = 0;
        relativeHeading = 0;
        offset = 0;
    }

    /**
     * Inverts the ouptut of gyro
     */
    public void invertGyro() {
        multiplier *= -1;
    }

    /**
     * @return Relative heading of the robot
     */
    public double getHeading() {
        // Return yaw
        return getAbsoluteHeading() - offset;
    }

    /**
     * @return Absolute heading of the robot
     */
    @Override
    public double getAbsoluteHeading() {
        return revIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) * multiplier;
    }

    /**
     * @return X, Y, Z angles of gyro
     */
    public double[] getAngles() {
        // make a singular hardware call
        YawPitchRollAngles orientation = revIMU.getRobotYawPitchRollAngles();

        return new double[]{orientation.getYaw(AngleUnit.DEGREES), orientation.getPitch(AngleUnit.DEGREES), orientation.getRoll(AngleUnit.DEGREES)};
    }

    /**
     * @return Transforms heading into {@link Rotation2d}
     */
    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void disable() {
        revIMU.close();
    }

    @Override
    public void reset() {
        offset += getHeading();
    }

    @Override
    public String getDeviceType() {
        return "Rev Expansion Hub IMU";
    }

    /**
     * @return the internal sensor being wrapped
     */
    public BHI260IMU getRevIMU() {
        return revIMU;
    }

}