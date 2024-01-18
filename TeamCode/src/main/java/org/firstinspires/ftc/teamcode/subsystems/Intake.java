package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake extends SubsystemBase {

    public enum IntakeState {
        Intaking,
        Idle,
        Reverse
    }


    public static double UP = 0d, DOWN = 0.37;
    public static double INTAKE_POWER = 0.8, EJECT_POWER = -0.6;
    public static double AMPS_THRESHOLD = 100d;
    public static double CHECK_INTERVAL = 0.6;

    DcMotorEx motor;
    Servo servo;
    IntakeState intakeState;
    ElapsedTime timer;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        servo = hardwareMap.get(Servo.class, "intakeServo");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.stopIntake();

        timer = new ElapsedTime();
        timer.reset();
    }

    public void startMotor() {
        motor.setPower(INTAKE_POWER);
    }
    public void stopMotor() {
        motor.setPower(0);
    }
    public void reverseMotor() {
        motor.setPower(EJECT_POWER);
    }
    public void reverseMotorMax() {motor.setPower(-1d);}

    public void goUp() {
        servo.setPosition(UP);
    }
    public void goDown() {
        servo.setPosition(DOWN);
    }

    public void startIntake() {
        this.intakeState = IntakeState.Intaking;
        this.startMotor();
        this.goDown();
    }
    public void stopIntake() {
        this.intakeState = IntakeState.Idle;
        this.stopMotor();
        this.goUp();
    }

    public void reverseIntake() {
        this.intakeState = IntakeState.Reverse;
        this.goDown();
        this.reverseMotor();
    }

    public void reverseMotorAuto() {
        motor.setPower(-0.3);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void periodic() {
        if (intakeState == IntakeState.Intaking) {
            if (timer.seconds() < CHECK_INTERVAL) {
                return;
            }

            if (getCurrent() > AMPS_THRESHOLD) {
                this.reverseMotor();
            } else {
                this.startMotor();
            }

            timer.reset();
        }
    }

}
