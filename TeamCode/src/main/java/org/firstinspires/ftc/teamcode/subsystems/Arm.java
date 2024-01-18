package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.function.Supplier;

@Config
public class Arm extends SubsystemBase {

    public enum ArmState {
        Intake, Idle, Low, Mid, High
    }

    private ArmState armState;
    public static double IDLE = 0.02;
    public static double INTAKE = 0.05, DEPOSIT_LOW = 0.45, DEPOSIT_MID = 0.5, DEPOSIT_HIGH = 0.69;
    public static double WRIST_INTAKE = 0.44, WRIST_DEPOSIT_MID = 0.48, WRIST_DEPOSIT_LOW = 0.3, WRIST_DEPOSIT_HIGH = 0.66;
    public static int CHANGE_HEIGHT = 150;

    ServoImplEx left, right;
    Supplier<Integer> liftPositionSupplier;
    ServoImplEx wrist;

    public Arm(HardwareMap hardwareMap, Supplier<Integer> lpos) {
        left = hardwareMap.get(ServoImplEx.class, "left");
        right = hardwareMap.get(ServoImplEx.class, "right");

//        left.setPwmRange(new PwmControl.PwmRange(505, 2495));
//        right.setPwmRange(new PwmControl.PwmRange(505, 2495));

        liftPositionSupplier = lpos;

        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        wrist.setPosition(WRIST_INTAKE);

        armState = ArmState.Intake;
    }

    public void setPosition(double position) {
        left.setPosition(position);
        right.setPosition((1.0 - position));
    }

    public void goIntake() {
        armState = ArmState.Intake;
    }

    public void goLow() {
        armState = ArmState.Low;
    }

    public void goMid() {
        armState = ArmState.Mid;
    }

    public void goHigh() {
        armState = ArmState.High;
    }

    @Override
    public void periodic() {

        int liftPosition = liftPositionSupplier.get();

        if (armState == ArmState.Intake && liftPosition > CHANGE_HEIGHT) {
            armState = ArmState.Idle;
        }
        if (armState == ArmState.Idle && liftPosition <= CHANGE_HEIGHT) {
            armState = ArmState.Intake;
        }

        switch (armState) {
            case Intake:
                this.setPosition(INTAKE);
                wrist.setPosition(WRIST_INTAKE);
                break;

            case Idle:
                this.setPosition(IDLE);
                wrist.setPosition(WRIST_INTAKE);
                break;

            case Low:
                this.setPosition(DEPOSIT_LOW);
                wrist.setPosition(WRIST_DEPOSIT_LOW);
                break;

            case Mid:
                this.setPosition(DEPOSIT_MID);
                wrist.setPosition(WRIST_DEPOSIT_MID);
                break;

            case High:
                this.setPosition(DEPOSIT_HIGH);
                wrist.setPosition(WRIST_DEPOSIT_HIGH);
                break;

        }
    }

}
