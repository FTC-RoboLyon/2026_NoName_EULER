package FRC_ALDNC.commands;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ff_distance;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.kp_distance;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.tolerance_distance;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.tolerence_rotation;

import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.SubSystem.Camera_subsystem;
import FRC_ALDNC.SubSystem.Drive_Train;

public class drive_to_distance extends CommandBase {
    private Drive_Train chassis;
    private Camera_subsystem cam;
    private double distance_target;
    private double error;
    public drive_to_distance (Drive_Train chassiS, Camera_subsystem came, double Distance_target){
        distance_target = Distance_target;
        chassis = chassiS;
        cam = came;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        if (cam.getActual_detection() != null){
            error = distance_target - cam.getActual_detection().ftcPose.y;
            if (Math.abs(error) < tolerance_distance) {
                error = 0;
            }
            chassis.drive(distance_target * ff_distance + error*kp_distance, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return error == 0;
    }
}
