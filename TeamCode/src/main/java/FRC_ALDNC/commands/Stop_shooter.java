package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Stop_shooter extends CommandBase {
    Shooter_Subsystem shooter_subsystem;
    public Stop_shooter(Shooter_Subsystem shooter){
        shooter_subsystem = shooter;
    }

    @Override
    public void initialize() {
        shooter_subsystem.setShooter_state(Shooter_Subsystem.WantedState.WAIT);

    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
