package FRC_ALDNC.Auto.Command;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ff_rotation;

import com.arcrobotics.ftclib.command.CommandBase;

//import ExercicesAntoineChatGPT.CameraSimple;
import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;
import FRC_ALDNC.Auto.Subsystems.NavXSubsystem;
import FRC_ALDNC.SubSystem.Camera_subsystem;

public class AlignCommand extends CommandBase{
    double ff_rotation = 0.11,p_rotation = 0.0025, erreurPos, turn, targetAngle;
    DriveSubsystem driveSubsystem;
    Camera_subsystem cameraSubsystem;
    public AlignCommand(DriveSubsystem driveSubsystem, Camera_subsystem cameraSubsystem){
        this.cameraSubsystem = cameraSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        targetAngle = cameraSubsystem.getBearing() == 100000 ? targetAngle : Math.toDegrees(driveSubsystem.getAngle())+cameraSubsystem.getBearing();
        erreurPos = targetAngle - Math.toDegrees(driveSubsystem.getAngle());
        if (erreurPos > 180) erreurPos -= 360;
        if (erreurPos < -180) erreurPos += 360;
        turn = erreurPos*ff_rotation + erreurPos*p_rotation;
        driveSubsystem.drive(0, turn);

    }
}
