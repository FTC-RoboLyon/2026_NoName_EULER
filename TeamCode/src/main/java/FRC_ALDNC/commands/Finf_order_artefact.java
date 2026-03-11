package FRC_ALDNC.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import FRC_ALDNC.ALDNC_container;
import FRC_ALDNC.SubSystem.Camera_subsystem;

public class Finf_order_artefact extends CommandBase {
    Camera_subsystem came;
    ALDNC_container robot;
    boolean is_Configured;
    public Finf_order_artefact (Camera_subsystem cam, ALDNC_container mrobot){
        robot = mrobot;
        came = cam;
        addRequirements(came);
    }

    @Override
    public void execute() {
        if (came.getDetections() != null){
            for (AprilTagDetection detection : came.getDetections()){
                if (detection.id == 21) {
                    robot.Determinate_order(ALDNC_container.Artefact_order.GPP);
                    is_Configured = true;
                }
                else if (detection.id == 22) {
                    robot.Determinate_order(ALDNC_container.Artefact_order.PGP);
                    is_Configured = true;
                }
                else if (detection.id == 23) {
                    robot.Determinate_order(ALDNC_container.Artefact_order.PPG);
                    is_Configured = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return is_Configured;
    }
}
