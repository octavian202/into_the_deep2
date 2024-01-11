package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.Intake.EjectIntake;
import org.firstinspires.ftc.teamcode.commands.Intake.StartIntake;
import org.firstinspires.ftc.teamcode.commands.Intake.StopIntake;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveRobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.hanging.HangingControl;
import org.firstinspires.ftc.teamcode.commands.lift.ManualControlCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HangingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.Map;

import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "main", group = "1")
public class Main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap, lift::getPosition);
        Deposit deposit = new Deposit(hardwareMap);
        AirplaneLauncher airplaneLauncher = new AirplaneLauncher(hardwareMap);
        HangingSubsystem hangingSubsystem = new HangingSubsystem(hardwareMap);


        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);


        // drivetrain

        Supplier<Double> coefSupplier = () -> Map.calculate(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), 0d, 1.0, 0.7, 1.0);
        drivetrain.setDefaultCommand(new DriveRobotCentricCommand(drivetrain, gp1::getLeftX, gp1::getLeftY, gp1::getRightX, coefSupplier));
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(drivetrain::resetIMU, drivetrain));

        // lift

        Supplier<Double> liftSupplier = () -> {
            double leftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), rightTrigger = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            if (leftTrigger > 0) {
                return -leftTrigger;
            }
            return rightTrigger;
        };

        ManualControlCommand manualControlCommand = new ManualControlCommand(lift, liftSupplier);
        lift.setDefaultCommand(manualControlCommand);
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(lift::resetEncoder, lift));

        // deposit

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(deposit::deposit, deposit));
        gp1.getGamepadButton(GamepadKeys.Button.A).whenInactive(new InstantCommand(deposit::stop, deposit));


        // intake

        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new StartIntake(intake, deposit));
        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new StopIntake(intake, deposit));
        gp2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new EjectIntake(intake, deposit));

        // arm

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(arm::toggle, arm));

        // airplane

        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).and(gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).whenActive(new InstantCommand(airplaneLauncher::launch, airplaneLauncher));


        // hanging

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(gp1.getGamepadButton(GamepadKeys.Button.Y)).whenActive(new InstantCommand(hangingSubsystem::releaseArms, hangingSubsystem));
        hangingSubsystem.setDefaultCommand(new HangingControl(hangingSubsystem, gp2::getLeftY));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

//        gamepad1.setLedColor(38, 51, 92, Gamepad.LED_DURATION_CONTINUOUS);
//        gamepad2.setLedColor(237, 185, 44, Gamepad.LED_DURATION_CONTINUOUS);

            gamepad1.setLedColor(0.149, 0.2, 0.3607, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0.9294, 0.7254, 0.1725, Gamepad.LED_DURATION_CONTINUOUS);

            CommandScheduler.getInstance().run();

            telemetry.addData("lift dir", liftSupplier.get());
            telemetry.addData("lift pos", lift.getPosition());
            telemetry.addData("heading", drivetrain.getHeading());
            telemetry.addData("intake current", intake.getCurrent());
            telemetry.addData("battery voltage", voltageSensor.getVoltage());
            telemetry.update();
        }

    }
}
