package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.lift.AutoControlCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LowExtendCommand extends SequentialCommandGroup {

    public LowExtendCommand(Lift lift, Arm arm, int target) {
        addCommands(
                new InstantCommand(() -> {lift.setTarget(target);}, lift),
                new WaitCommand(1300),
                new InstantCommand(arm::goLow, arm)
        );

        addRequirements(arm);
    }

}
