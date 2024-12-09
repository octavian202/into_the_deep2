package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "dead wheel test", group = "debug")
public class DeadWheelsTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        OverflowEncoder encoderPar0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "lb")));
        OverflowEncoder encoderPar1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rb")));
        OverflowEncoder encoderPerp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rf")));

        encoderPar1.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("par 0", encoderPar0.getPositionAndVelocity().position);
            telemetry.addData("par 1", encoderPar1.getPositionAndVelocity().position);
            telemetry.addData("perp", encoderPerp.getPositionAndVelocity().position);
            telemetry.update();
        }
    }
}
