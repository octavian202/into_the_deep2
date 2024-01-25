package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BasicIMU;

public class Drivetrain extends SubsystemBase {

    public MotorEx frontLeft, frontRight, backRight, backLeft;
    MecanumDrive mecanumDrive;
//    BasicIMU imu;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft = new MotorEx(hardwareMap, "lf", Motor.GoBILDA.RPM_435);
        frontRight = new MotorEx(hardwareMap, "rf", Motor.GoBILDA.RPM_435);
        backRight = new MotorEx(hardwareMap, "rb", Motor.GoBILDA.RPM_435);
        backLeft = new MotorEx(hardwareMap, "lb", Motor.GoBILDA.RPM_435);

        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);
        backRight.setInverted(true);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);

        mecanumDrive = new MecanumDrive(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
//        imu = new BasicIMU(hardwareMap);
//        resetIMU();
    }

//    public double getHeading() {
//        return imu.getHeading();
//    }

//    public void resetIMU() {
//        imu.reset();
//    }

    public void driveRobotCentric(double x, double y, double rx, double coef) {
        x = x * coef;
        y = y * coef;
        rx = rx * coef;

        mecanumDrive.driveRobotCentric(x, y, rx, false);
    }

//    public void driveFieldCentric(double x, double y, double rx, double coef) {
//        x = x * coef;
//        y = y * coef;
//        rx = rx * coef;
//
//        mecanumDrive.driveFieldCentric(x, y, rx, imu.getRotation2d().getDegrees(), false);
//    }


}
