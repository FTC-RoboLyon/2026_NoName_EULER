package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Regler_shooter_distance extends CommandBase {
    Shooter_Subsystem shhooter;
    Camera_subsystem came;
    public Regler_shooter_distance(Shooter_Subsystem Shooter, Camera_subsystem Cam){
        shhooter = Shooter;
        came = Cam;
        addRequirements(shhooter);
    }

    @Override
    public void initialize() {
        shhooter.calculate_postir(came.getActual_detection().ftcPose.y);
        shhooter.setShooter_state(Shooter_Subsystem.WantedState.AUTO);}


    @Override
    public void end(boolean interrupted) {
        shhooter.setShooter_state(Shooter_Subsystem.WantedState.WAIT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
