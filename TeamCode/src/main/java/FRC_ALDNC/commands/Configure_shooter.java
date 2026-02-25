package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Configure_shooter extends CommandBase {
    Shooter_Subsystem shooter;

    Shooter_Subsystem.WantedState postir;
    Camera_subsystem came;
    boolean toggle;
    boolean isConfigured;
    public Configure_shooter(Shooter_Subsystem shooter, Camera_subsystem cam, Shooter_Subsystem.WantedState postir, boolean toggle){
        this.toggle = toggle;
        came = cam;
        this.shooter = shooter;
        this.postir = postir;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (postir == Shooter_Subsystem.WantedState.AUTO ) {
            if (came.getActual_detection() != null) {
                shooter.calculate_postir(came.getActual_detection().ftcPose.y);
                isConfigured = true;
                shooter.setShooter_state(postir);
            }
        } else{
            isConfigured = true;
            shooter.setShooter_state(postir);
        }
    }

    @Override
    public void execute() {
        if (isConfigured)
            return;
        else {
            if (came.getActual_detection() != null) {
                shooter.calculate_postir(came.getActual_detection().ftcPose.y);
                isConfigured = true;
                shooter.setShooter_state(postir);
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (toggle)
            shooter.setShooter_state(Shooter_Subsystem.WantedState.WAIT);
    }

    @Override
    public boolean isFinished() {
        if (!toggle)
            return shooter.getShooterSysState() == Shooter_Subsystem.SystemState.READY_TO_SHOOT;
        return false;
    }
}
