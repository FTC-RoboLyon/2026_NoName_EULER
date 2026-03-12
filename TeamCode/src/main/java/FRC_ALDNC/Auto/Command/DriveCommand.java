package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import FRC_ALDNC.Auto.Container;
import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    double x,y;
    Gamepad gamepad1;
    DriveSubsystem driveSubsystem;
    Container container;
    public DriveCommand(DriveSubsystem driveSubsystem, Container container, Gamepad gamepad1) {
        this.driveSubsystem = driveSubsystem;
        this.container = container;
        this.gamepad1 = gamepad1;
        addRequirements(driveSubsystem);
    }
    @Override
    public void execute(){
        driveSubsystem.drive(gamepad1);
    }
}
