package org.firstinspires.ftc.teamcode.commands.lift;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.function.Supplier;

@Config
public class ManualControlCommand extends CommandBase {
    Lift lift;
    Supplier<Double> directionSupplier;
    public static double step = 60;

    public ManualControlCommand(Lift lift, Supplier<Double> sup) {
        this.lift = lift;
        this.directionSupplier = sup;

        addRequirements(lift);
    }

    @Override
    public void execute() {
        int newTarget = lift.getTarget() + (int)(step * directionSupplier.get());
        lift.setTarget(newTarget);
    }

}
