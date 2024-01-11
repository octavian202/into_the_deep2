package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DriveFieldCentricCommand extends CommandBase {

    Drivetrain drivetrain;
    Supplier<Double> xSupplier, ySupplier, rxSupplier, coefSupplier;


    public DriveFieldCentricCommand(Drivetrain drivetrain,Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rxSupplier, Supplier<Double> coefSupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rxSupplier = rxSupplier;
        this.coefSupplier = coefSupplier;

        addRequirements(drivetrain);
    }

    public DriveFieldCentricCommand(Drivetrain drivetrain,Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rxSupplier) {
        this(drivetrain, xSupplier, ySupplier, rxSupplier, () -> 1.0);
    }

    @Override
    public void execute() {
        drivetrain.driveFieldCentric(xSupplier.get(), ySupplier.get(), rxSupplier.get(), coefSupplier.get());
    }


}
