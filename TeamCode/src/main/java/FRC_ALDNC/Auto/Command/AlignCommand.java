package FRC_ALDNC.Auto.Command;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ff_rotation;

import com.arcrobotics.ftclib.command.CommandBase;

import ExercicesAntoineChatGPT.CameraSimple;
import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;
import FRC_ALDNC.Auto.Subsystems.NavXSubsystem;
import FRC_ALDNC.SubSystem.Camera_subsystem;

public class AlignCommand extends CommandBase{
    double ff_rotation = 0.11,p_rotation = 0.0025, erreurPos, turn;
    DriveSubsystem driveSubsystem;
    Camera_subsystem cameraSubsystem;
    NavXSubsystem navx;
    public AlignCommand(DriveSubsystem driveSubsystem, Camera_subsystem cameraSubsystem, NavXSubsystem navx){
        this.navx = navx;
        this.cameraSubsystem = cameraSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        erreurPos = cameraSubsystem.getBearing() == 100000 ? erreurPos : Math.toDegrees(navx.getAngle())-cameraSubsystem.getBearing();
        turn = erreurPos*ff_rotation + erreurPos*p_rotation;
        driveSubsystem.drive(0, turn);

    }
}
