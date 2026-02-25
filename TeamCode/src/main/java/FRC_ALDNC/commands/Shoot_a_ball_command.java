package FRC_ALDNC.commands;


import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posFeed;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import FRC_ALDNC.SubSystem.Feeder_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import lib.Utils;

public class Shoot_a_ball_command extends CommandBase {

    Feeder_subsystem feeder;
    Shooter_Subsystem shooter;
    public Shoot_a_ball_command ( Feeder_subsystem feeder, Shooter_Subsystem shooter){
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements( feeder);
    }

    @Override
    public void initialize() {
        feeder.setfeeder_wanted_state(Feeder_subsystem.Feeder_wanted_state.Haut);
    }

    @Override
    public boolean isFinished() {
        //return Utils.IsInRange(feeder.get_Pos(), posFeed, 0.005);
        return shooter.has_shoot();
    }
}
