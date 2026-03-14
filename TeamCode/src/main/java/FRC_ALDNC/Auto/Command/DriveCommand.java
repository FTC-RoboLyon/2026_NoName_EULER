package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    Gamepad gamepad1;
    DriveSubsystem driveSubsystem;
    public DriveCommand(DriveSubsystem driveSubsystem, Gamepad gamepad1) {
        this.driveSubsystem = driveSubsystem;
        this.gamepad1 = gamepad1;
        addRequirements(driveSubsystem);
    }
    @Override
    public void execute(){
        driveSubsystem.drive(gamepad1);
    }
}
