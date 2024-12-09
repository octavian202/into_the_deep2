package org.firstinspires.ftc.teamcode.commands.hanging;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HangingSubsystem;

import java.util.function.Supplier;

public class HangingControl extends CommandBase {

    HangingSubsystem hangingSubsystem;
    Supplier<Double> directionSupplier;

    public HangingControl(HangingSubsystem hangingSubsystem, Supplier<Double> directionSupplier) {
        this.hangingSubsystem = hangingSubsystem;
        this.directionSupplier = directionSupplier;

        addRequirements(hangingSubsystem);
    }

    @Override
    public void execute() {
        hangingSubsystem.setPower(directionSupplier.get());
    }

}
