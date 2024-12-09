package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HangingSubsystem extends SubsystemBase {

    private ServoImplEx left, right;
    private DcMotorEx motor;
    private boolean isUp = false;

    public HangingSubsystem(HardwareMap hardwareMap) {

        motor = hardwareMap.get(DcMotorEx.class, "hang");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left = hardwareMap.get(ServoImplEx.class, "lhang");
        right = hardwareMap.get(ServoImplEx.class, "rhang");

        left.setPosition(0.5);
        right.setPosition(0.5);

        isUp = false;
    }

    public void releaseArms() {
        left.setPosition(0);
        right.setPosition(1.0);
        isUp = true;
    }

    public void setPower(double power) {
        if (!isUp)
            return;

        motor.setPower(power);
    }

}
