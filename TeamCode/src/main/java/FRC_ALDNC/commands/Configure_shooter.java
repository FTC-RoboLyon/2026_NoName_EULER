package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Configure_shooter extends CommandBase {
    Shooter_Subsystem shooter;

    Shooter_Subsystem.WantedState postir;
    public Configure_shooter(Shooter_Subsystem shooter, Shooter_Subsystem.WantedState postir){
        this.shooter = shooter;
        this.postir = postir;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooter_state(postir);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
