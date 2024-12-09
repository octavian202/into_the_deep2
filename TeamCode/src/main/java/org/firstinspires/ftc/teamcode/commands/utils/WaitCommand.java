package org.firstinspires.ftc.teamcode.commands.utils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitCommand extends CommandBase {

    ElapsedTime timer;
    int time;
    public WaitCommand(int ms) {
        this.time = ms;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return (timer.milliseconds() >= time);
    }

}
