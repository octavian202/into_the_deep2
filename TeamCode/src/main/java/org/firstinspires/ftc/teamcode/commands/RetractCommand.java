package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.lift.AutoControlCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class RetractCommand extends SequentialCommandGroup {

    public static int WAIT_MS = 1000;

    public RetractCommand(Lift lift, Arm arm) {
        addCommands(
                new InstantCommand(arm::goIntake, arm),
                new WaitCommand(WAIT_MS),
                new InstantCommand(() -> lift.setTarget(0), lift)
        );

        addRequirements(arm);
    }
}
