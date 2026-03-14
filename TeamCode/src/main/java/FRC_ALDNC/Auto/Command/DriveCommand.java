package FRC_ALDNC.Auto.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private double targetAngle = 0;
    private double x, y;
    Gamepad gamepad1;
    DriveSubsystem driveSubsystem;
    public DriveCommand(DriveSubsystem driveSubsystem, Gamepad gamepad1) {
        this.driveSubsystem = driveSubsystem;
        this.gamepad1 = gamepad1;
        addRequirements(driveSubsystem);
    }
    @Override
    public void execute(){
        if (gamepad1.xWasPressed()){
            targetAngle += 0.1;
        }else if(gamepad1.bWasPressed()){
            targetAngle -= 0.1;
        }
        //driveSubsystem.drive(gamepad1);
        driveSubsystem.goAngle(x, y);
    }
}
