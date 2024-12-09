package org.firstinspires.ftc.teamcode.commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class StopIntake extends CommandBase {
    Intake intake;
    Deposit deposit;

    public StopIntake(Intake intake, Deposit deposit) {
        this.intake = intake;
        this.deposit = deposit;

        addRequirements(intake, deposit);
    }

    @Override
    public void initialize() {
        intake.stopIntake();
        deposit.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}