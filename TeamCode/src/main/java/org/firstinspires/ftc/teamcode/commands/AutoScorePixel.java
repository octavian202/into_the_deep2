package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class AutoScorePixel extends SequentialCommandGroup {

    public AutoScorePixel(Lift lift, Arm arm, Deposit deposit) {
        addCommands(
                new InstantCommand(deposit::deposit, deposit),
                new WaitCommand(1000),
                new InstantCommand(deposit::stop, deposit),
                new RetractCommand(lift, arm)
        );
    }
}
