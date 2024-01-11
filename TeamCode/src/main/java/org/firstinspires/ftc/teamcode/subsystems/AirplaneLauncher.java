package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class AirplaneLauncher extends SubsystemBase {

    public static double UP = 0.36, DOWN = 0.42;

    private ServoImplEx servo;

    public AirplaneLauncher(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "plane");
//        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));

        enable();
        servo.setPosition(DOWN);
    }

    public void enable() {
        servo.setPwmEnable();
    }

    public void disable() {
        servo.setPwmDisable();
    }

    public void launch() {
        enable();
        servo.setPosition(UP);
    }

}
