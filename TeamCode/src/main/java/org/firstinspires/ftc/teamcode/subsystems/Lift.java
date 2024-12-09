package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Lift extends SubsystemBase {

    public static double KP = 0.015, KI = 0, KD = 0;

    private DcMotorEx motor;
    public static final double POWER_MIN = -0.6, POWER_MAX = 1.0;
    public static int target = 0;
    PIDController pidController;

    

    public Lift(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "sliders");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        offset = 0;
        target = 0;

        pidController = new PIDController(KP, KI, KD);

    }

    private int offset = 0;
    private int position = 0;
    private void read() {
        position = motor.getCurrentPosition() + offset;
    }
    public int getPosition() {
        return position;
    }
    public void resetEncoder() {
        offset = -motor.getCurrentPosition();
    }


    public void setPower(double power) {

        power = Math.max(POWER_MIN, power);
        power = Math.min(POWER_MAX, power);

        motor.setPower(power);
    }

    public void setTarget(int target) {
        this.target = target;
    }
    public int getTarget() {
        return this.target;
    }

    @Override
    public void periodic() {
        read();
        pidController.setPID(KP, KI, KD);

        setTarget(Math.max(0, getTarget()));
        setTarget(Math.min(1820, getTarget()));

        double output = pidController.calculate(this.getPosition(), target);
        this.setPower(output);

    }

}
