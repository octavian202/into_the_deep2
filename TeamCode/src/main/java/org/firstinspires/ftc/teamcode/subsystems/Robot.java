package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Path;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.movement.MecanumDrive;

public class Robot {

    public enum OpModeType {
        Auto,
        TeleOp
    }

    AirplaneLauncher airplaneLauncher;
    Arm arm;
    Deposit deposit;
    Drivetrain drivetrain;
    MecanumDrive mecanumDrive;
    HangingSubsystem hangingSubsystem;
    Intake intake;
    Lift lift;


    public Robot(HardwareMap hardwareMap, OpModeType opModeType) {

        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap, lift::getPosition);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap);

        lift.resetEncoder();


        if (opModeType == OpModeType.Auto) {
            mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        } else {
            drivetrain = new Drivetrain(hardwareMap);
            airplaneLauncher = new AirplaneLauncher(hardwareMap);
            hangingSubsystem = new HangingSubsystem(hardwareMap);
        }
    }

}
