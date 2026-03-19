package FRC_ALDNC.commands;

import static FRC_ALDNC.CONSTANT.constante_feeder.time_to_let_a_ball_pass;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        feeder.setfeeder_wanted_state(Feeder_subsystem.Feeder_wanted_state.Haut);

        timer = new ElapsedTime();
        timer.reset();

    }

    @Override
    public void end(boolean interrupted) {
        feeder.setfeeder_wanted_state(Feeder_subsystem.Feeder_wanted_state.bas);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= 250;
    }
}
