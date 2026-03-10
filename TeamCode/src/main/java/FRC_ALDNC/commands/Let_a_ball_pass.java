package FRC_ALDNC.commands;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.time_to_let_a_ball_pass;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Feeder_subsystem;


public class Let_a_ball_pass extends CommandBase {
    Feeder_subsystem feeder;
    ElapsedTime timer;
    boolean isfinished;
    Telemetry telemetrY;
    public Let_a_ball_pass(Feeder_subsystem feeder, Telemetry telemetry){
        telemetrY = telemetry;
        this.feeder = feeder;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setfeeder_wanted_state(Feeder_subsystem.Feeder_wanted_state.bas);

        timer = new ElapsedTime();
        timer.reset();
        telemetrY.addLine("let a ball is initialize ");
        telemetrY.update();
    }

    @Override
    public void execute() {

        telemetrY.addLine("let a ball has execute ");
        telemetrY.update();
    }

    @Override
    public void end(boolean interrupted) {
        telemetrY.addLine("let a ball has finish ");
        telemetrY.update();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= time_to_let_a_ball_pass;
    }
}
