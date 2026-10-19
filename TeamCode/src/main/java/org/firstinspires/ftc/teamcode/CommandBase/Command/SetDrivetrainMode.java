package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Drivetrain.DriveTrainSubsystem;

public class SetDrivetrainMode extends CommandBase {
    private DriveTrainSubsystem drivetrain;
    private DriveTrainSubsystem.DriveMode driveMode;
    public SetDrivetrainMode (DriveTrainSubsystem driveTrainSubsystem, DriveTrainSubsystem.DriveMode driveMode){
        this.drivetrain = driveTrainSubsystem;
        this.driveMode = driveMode;
    }

    @Override
    public void initialize(){
        drivetrain.setDriveMode(driveMode);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
