package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import Webcam_aldnc_yeux.Apriltag_reader;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

public class Tuning_postir_command extends CommandBase {
    Shooter_Subsystem shooter;
    Camera_subsystem apriljoke;
    public boolean Cannot_tune_sans_april;
    public Tuning_postir_command (Shooter_Subsystem shooter_subsystem, Camera_subsystem cam){
        shooter = shooter_subsystem;
        apriljoke = cam;
        addRequirements(shooter, apriljoke);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (apriljoke.getActual_detection() == null)
            Cannot_tune_sans_april = true;
        else
            Cannot_tune_sans_april = false;
        shooter.calculate_postir(apriljoke.getActual_detection().ftcPose.x);
    }

    @Override
    public boolean isFinished() {
        return Cannot_tune_sans_april;
    }
}
