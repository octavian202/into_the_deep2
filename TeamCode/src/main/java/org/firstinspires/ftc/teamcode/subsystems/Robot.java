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

    public AirplaneLauncher airplaneLauncher;
    public Arm arm;
    public Deposit deposit;
    public Drivetrain drivetrain;
    public MecanumDrive mecanumDrive;
    public HangingSubsystem hangingSubsystem;
    public Intake intake;
    public Lift lift;


    public Robot(HardwareMap hardwareMap, OpModeType opModeType) {

        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap, lift::getPosition);
        deposit = new Deposit(hardwareMap);
        intake = new Intake(hardwareMap);
        airplaneLauncher = new AirplaneLauncher(hardwareMap);
        hangingSubsystem = new HangingSubsystem(hardwareMap);

        lift.resetEncoder();


        if (opModeType == OpModeType.Auto) {
            mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        } else {
            drivetrain = new Drivetrain(hardwareMap);
        }
    }

}
