package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import FRC_ALDNC.Auto.Container;
import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    double x,y;
    DriveSubsystem driveSubsystem;
    Container container;
    public DriveCommand(DriveSubsystem driveSubsystem, Container container){
        this.driveSubsystem = driveSubsystem;
        this.container = container;
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        x = container.getX();
        y = container.getY();
        driveSubsystem.drive(x,y);
    }
}
