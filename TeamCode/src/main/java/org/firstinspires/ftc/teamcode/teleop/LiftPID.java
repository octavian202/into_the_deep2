package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.lift.AutoControlCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@TeleOp(name = "lift pid tuning", group = "tuning")
public class LiftPID extends LinearOpMode {

    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Lift lift = new Lift(hardwareMap);
        AutoControlCommand autoControlCommand = new AutoControlCommand(lift);
        lift.setDefaultCommand(autoControlCommand);

        Arm arm = new Arm(hardwareMap, lift::getPosition);
        arm.goIntake();

        waitForStart();

        while (opModeIsActive()) {
            autoControlCommand.setTarget(target);
            CommandScheduler.getInstance().run();

            telemetry.addData("pos", lift.getPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }

    }
}
