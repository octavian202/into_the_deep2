package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.function.Supplier;

@Config
public class Arm extends SubsystemBase {

    public enum ArmState {
        Intake, Idle, Low, Mid, High, Stack
    }

    private ArmState armState;
    public static double IDLE = 0.025;
    public static double INTAKE = 0.06, DEPOSIT_LOW = 0.32, DEPOSIT_MID = 0.52, DEPOSIT_HIGH = 0.71;
    public static double WRIST_INTAKE = 0.42, WRIST_DEPOSIT_MID = 0.48, WRIST_DEPOSIT_LOW = 0.3, WRIST_DEPOSIT_HIGH = 0.72;
    public static double WRIST_INTAKE_STACK = 0.395;
    public static int CHANGE_HEIGHT = 150, SAFE_HEIGHT = 900;

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
    public void goStack() {armState = ArmState.Stack;}

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

        switch (armState) {
            case Intake:
                this.setPosition(INTAKE);
                wrist.setPosition(WRIST_INTAKE);

                if (liftPosition > CHANGE_HEIGHT) {
                    armState = ArmState.Idle;
                }

                break;
            case Stack:
                this.setPosition(INTAKE);
                wrist.setPosition(WRIST_INTAKE_STACK);

                if (liftPosition > CHANGE_HEIGHT) {
                    armState = ArmState.Idle;
                }
                break;

            case Idle:
                this.setPosition(IDLE);
                wrist.setPosition(WRIST_INTAKE);

                if (liftPosition <= CHANGE_HEIGHT) {
                    armState = ArmState.Intake;
                }
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


            default:
                break;
        }
    }

}
