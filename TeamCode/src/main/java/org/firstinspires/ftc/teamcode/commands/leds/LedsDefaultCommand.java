package org.firstinspires.ftc.teamcode.commands.leds;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Leds;

public class LedsDefaultCommand extends CommandBase {

    final int FINAL_SECONDS = 114;
    Leds leds;
    ElapsedTime timer;

    public LedsDefaultCommand(Leds leds, Deposit deposit) {
        this.leds = leds;

        addRequirements(leds, deposit);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        if (timer.seconds() >= FINAL_SECONDS) {
            leds.setColor("ambulance");
            return;
        }

        // citeste culoarea de la pixelul din deposit
        leds.setColor("blue");
    }

    public void resetTimer() {
        timer.reset();
    }

}
