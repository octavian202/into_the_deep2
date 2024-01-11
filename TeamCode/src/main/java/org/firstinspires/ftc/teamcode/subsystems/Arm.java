package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.function.Supplier;

@Config
public class Arm extends SubsystemBase {

    public static double INTAKE = 0.05, DEPOSIT = 0.5, IDLE = 0.02;
    public static double WRIST_INTAKE = 0.44, WRIST_DEPOSIT = 0.48;
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

        isUp = false;
        low = false;
    }

    public void setPosition(double position) {
        left.setPosition(position);
        right.setPosition((1.0 - position));
    }

    public boolean isUp = false;
    public boolean low = false;
    public void toggle() {
        isUp = !isUp;
    }

    public void goDown() {
        isUp = false;
        low = false;
    }
    public void goUp() {
        low = false;
        isUp = true;
    }
    public boolean getIsUp() {return isUp;}
    public void goLow(){
        goUp();
        low = true;
    }

    @Override
    public void periodic() {

        int liftPosition = liftPositionSupplier.get();

        if (isUp) {
            if (low) {
                this.setPosition(0.25);
                wrist.setPosition(0.3);
            } else {
                this.setPosition(DEPOSIT);
                wrist.setPosition(WRIST_DEPOSIT);
            }
        } else {
            wrist.setPosition(WRIST_INTAKE);

            if (liftPosition <= CHANGE_HEIGHT) {
                this.setPosition(INTAKE);
            } else {
                this.setPosition(IDLE);
            }
        }
    }

}
