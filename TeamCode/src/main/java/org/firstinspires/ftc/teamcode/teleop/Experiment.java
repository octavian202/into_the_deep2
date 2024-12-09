package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveRobotCentricCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.List;

@TeleOp(name = "experiment", group = ".")
public class Experiment extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "hang");
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "lf");
        Intake intake = new Intake(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        Motor.Encoder leftOdo = (new Motor(hardwareMap, "lb", Motor.GoBILDA.RPM_435)).encoder;
        Motor.Encoder rightOdo = (new Motor(hardwareMap, "rb", Motor.GoBILDA.RPM_435)).encoder;

        leftOdo.reset();
        rightOdo.reset();

        GamepadEx gp1 = new GamepadEx(gamepad1);

        MotorEx frontLeft, frontRight, backRight, backLeft;
        frontLeft = new MotorEx(hardwareMap, "lf", Motor.GoBILDA.RPM_435);
        frontRight = new MotorEx(hardwareMap, "rf", Motor.GoBILDA.RPM_435);
        backRight = new MotorEx(hardwareMap, "rb", Motor.GoBILDA.RPM_435);
        backLeft = new MotorEx(hardwareMap, "lb", Motor.GoBILDA.RPM_435);

        frontLeft.setInverted(true);
        frontRight.setInverted(false);
        backLeft.setInverted(true);
        backRight.setInverted(false);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(intake::stopIntake, intake));


        boolean found = false;
        double leftVal = 0, rightVal = 0;
        double time = 0;
        boolean running = false;
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();



            if (gamepad1.triangle) {
                timer.reset();
                running = true;
                frontLeft.set(1.0);
                frontRight.set(1.0);
                backLeft.set(1.0);
                backRight.set(1.0);
            }

            if (running) {
                if (timer.seconds() > 1.5) {
                    running = false;
                    frontLeft.set(0d);
                    frontRight.set(0d);
                    backLeft.set(0d);
                    backRight.set(0d);
                }

                if (Math.abs(left.getVelocity()) >= 2300 && Math.abs(right.getVelocity()) >= 2300 && !found) {
                    found = true;
                    leftVal = leftOdo.getPosition() * 0.00053442379;
                    rightVal = rightOdo.getPosition() * 0.00053442379;
                    time = timer.seconds();
                    telemetry.addData("time", time);
                    telemetry.addData("left odo", leftOdo.getPosition() * 0.00053442379);
                    telemetry.addData("right odo", rightOdo.getPosition() * 0.00053442379);
                    telemetry.update();
                }
            }
        }

    }
}
