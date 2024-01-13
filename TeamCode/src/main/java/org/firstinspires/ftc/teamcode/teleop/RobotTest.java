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

import org.firstinspires.ftc.teamcode.commands.Intake.EjectIntake;
import org.firstinspires.ftc.teamcode.commands.Intake.StartIntake;
import org.firstinspires.ftc.teamcode.commands.Intake.StopIntake;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveRobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.hanging.HangingControl;
import org.firstinspires.ftc.teamcode.commands.lift.ManualControlCommand;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utils.Map;

import java.util.List;
import java.util.function.Supplier;

@TeleOp(name = "main test", group = ".")
public class RobotTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        Robot robot = new Robot(hardwareMap, Robot.OpModeType.TeleOp);

        // drivetrain

        Supplier<Double> coefSupplier = () -> Map.calculate(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), 0d, 1.0, 0.7, 1.0);
        robot.drivetrain.setDefaultCommand(new DriveRobotCentricCommand(robot.drivetrain, gp1::getLeftX, gp1::getLeftY, gp1::getRightX, coefSupplier));
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(robot.drivetrain::resetIMU, robot.drivetrain));

        // lift

        Supplier<Double> liftSupplier = () -> {
            double leftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), rightTrigger = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            if (leftTrigger > 0) {
                return -leftTrigger;
            }
            return rightTrigger;
        };

        ManualControlCommand manualControlCommand = new ManualControlCommand(robot.lift, liftSupplier);
        robot.lift.setDefaultCommand(manualControlCommand);
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(robot.lift::resetEncoder, robot.lift));

        // deposit

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(robot.deposit::deposit, robot.deposit));
        gp1.getGamepadButton(GamepadKeys.Button.A).whenInactive(new InstantCommand(robot.deposit::stop, robot.deposit));


        // intake

        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new StartIntake(robot.intake, robot.deposit));
        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new StopIntake(robot.intake, robot.deposit));
        gp2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new EjectIntake(robot.intake, robot.deposit));

        // arm

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(robot.arm::toggle, robot.arm));

        // airplane

        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).and(gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)).whenActive(new InstantCommand(robot.airplaneLauncher::launch, robot.airplaneLauncher));


        // hanging

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(gp1.getGamepadButton(GamepadKeys.Button.Y)).whenActive(new InstantCommand(robot.hangingSubsystem::releaseArms, robot.hangingSubsystem));
        robot.hangingSubsystem.setDefaultCommand(new HangingControl(robot.hangingSubsystem, gp2::getLeftY));


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("intake current", robot.intake.getCurrent());
            telemetry.addData("lift pos", robot.lift.getPosition());
            telemetry.addData("lift target", robot.lift.getTarget());
            telemetry.update();
        }

    }
}
