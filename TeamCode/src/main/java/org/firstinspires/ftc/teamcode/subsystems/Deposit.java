package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Deposit extends SubsystemBase {
    CRServo servo;

    public Deposit(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "deposit");

        this.stop();
    }

    public void intake() {
        servo.setPower(1.0);
    }
    public void stop() {
        servo.setPower(0);
    }
    public void deposit() {
        servo.setPower(-1.0);
    }
}
