package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Leds extends SubsystemBase {

    RevBlinkinLedDriver revBlinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public Leds(HardwareMap hardwareMap) {
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        revBlinkinLedDriver.setPattern(pattern);
    }

    public void setColor(String color) {
        switch (color) {
            case "green":
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case "violet":
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
            case "yellow":
                pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case "ambulance":
                pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                break;
            default:
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                break;
        }
    }

    @Override
    public void periodic() {
        revBlinkinLedDriver.setPattern(pattern);
    }

}
