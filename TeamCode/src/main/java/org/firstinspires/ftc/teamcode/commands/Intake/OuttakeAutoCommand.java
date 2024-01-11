package org.firstinspires.ftc.teamcode.commands.Intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class OuttakeAutoCommand extends SequentialCommandGroup {

    public static int OUTTAKE_MS = 1000, WAIT_TO_GO_DOWN = 500;

    public OuttakeAutoCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::goDown, intake),
                new WaitCommand(WAIT_TO_GO_DOWN),
                new InstantCommand(intake::reverseMotorAuto),
                new WaitCommand(OUTTAKE_MS),
                new InstantCommand(intake::stopIntake)
        );

        addRequirements(intake);
    }

}
