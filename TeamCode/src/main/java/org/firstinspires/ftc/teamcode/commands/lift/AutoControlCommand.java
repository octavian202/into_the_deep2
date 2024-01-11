package org.firstinspires.ftc.teamcode.commands.lift;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class AutoControlCommand extends CommandBase {

    Lift lift;
    private int target = 0;

    public AutoControlCommand(Lift lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    @Override
    public void execute() {
        lift.setTarget(target);
    }
}
