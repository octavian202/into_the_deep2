package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "servo test", group = "tuning")
public class ServoTest extends LinearOpMode {

    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "plane");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(target);
        }
    }
}
