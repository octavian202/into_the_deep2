package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Config
@TeleOp(name = "arm test", group = "tuning")
public class ArmTest extends LinearOpMode {
    public static double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm arm = new Arm(hardwareMap, () -> 0);

        waitForStart();

        while (opModeIsActive()) {
            arm.setPosition(target);
        }
    }
}
