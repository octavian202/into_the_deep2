package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Leds;

@TeleOp(name = "leds test", group = "debug")
public class LedsTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        GamepadEx gp1 = new GamepadEx(gamepad1);

        Leds leds = new Leds(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
