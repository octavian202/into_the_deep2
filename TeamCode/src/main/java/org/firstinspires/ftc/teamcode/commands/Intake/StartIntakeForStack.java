package org.firstinspires.ftc.teamcode.commands.Intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class StartIntakeForStack extends CommandBase {
    Intake intake;
    Deposit deposit;
    Arm arm;

    public StartIntakeForStack(Intake intake, Deposit deposit, Arm arm) {
        this.intake = intake;
        this.deposit = deposit;
        this.arm = arm;

        addRequirements(intake, deposit, arm);
    }

    @Override
    public void initialize() {
        intake.startIntakeForStack();
        deposit.intake();
        arm.goStack();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
