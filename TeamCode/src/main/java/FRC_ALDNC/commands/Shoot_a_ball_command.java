package FRC_ALDNC.commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Shoot_a_ball_command extends CommandBase {
    Shooter_Subsystem shooter;
    Feeder_subsystem feeder;
    public Shoot_a_ball_command (Shooter_Subsystem shooter, Feeder_subsystem feeder){
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        feeder.setfeeder_wanted_state(Feeder_subsystem.Feeder_wanted_state.Haut);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
