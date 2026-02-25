package FRC_ALDNC.commands;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.time_to_let_a_ball_pass;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import FRC_ALDNC.SubSystem.Feeder_subsystem;


public class Let_a_ball_pass extends CommandBase {
    Feeder_subsystem feeder;
    ElapsedTime timer;
    boolean isfinished;
    public Let_a_ball_pass(Feeder_subsystem feeder){

        this.feeder = feeder;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setfeeder_wanted_state(Feeder_subsystem.Feeder_wanted_state.bas);
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        isfinished = timer.milliseconds() >= time_to_let_a_ball_pass;
    }

    @Override
    public boolean isFinished() {
        return isfinished;
    }
}
