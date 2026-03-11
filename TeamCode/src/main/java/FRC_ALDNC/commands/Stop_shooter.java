package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Stop_shooter extends CommandBase {
    Shooter_Subsystem shooter_subsystem;
    Telemetry telemetrY;
    public Stop_shooter(Shooter_Subsystem shooter, Telemetry telemetry){
        telemetrY = telemetry;
        shooter_subsystem = shooter;
    }

    @Override
    public void initialize() {
        shooter_subsystem.setShooter_state(Shooter_Subsystem.WantedState.WAIT);
        telemetrY.addLine("stop shooter has initialize");
        telemetrY.update();
    }

    @Override
    public void execute() {
        telemetrY.addLine("stop shooter has execute");
        telemetrY.update();
    }

    @Override
    public void end(boolean interrupted) {
        telemetrY.addLine("stop shooter has finished");
        telemetrY.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
