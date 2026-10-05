package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics.Pose2d;

public class GoToPosCommand extends CommandBase {

    DriveTrainSubsystem driveTrainSubsystem;
    Pose2d posTarget;

    public GoToPosCommand(DriveTrainSubsystem driveTrain, Pose2d pose2d) {
        driveTrainSubsystem = driveTrain;
        posTarget = pose2d;
        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.setDriveMode(DriveTrainSubsystem.DriveMode.GO_TO_POS);
        driveTrainSubsystem.setGoToPosTargets(posTarget.xMeters, posTarget.yMeters, posTarget.headingRadians);
    }


    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.setDriveMode(DriveTrainSubsystem.DriveMode.DISABLE);
    }

    @Override
    public boolean isFinished() {
        return driveTrainSubsystem.isAtXYTargets();
    }//donc tu ne prends pas en compte le heading dans ton isFinished (c peut etre volontaire mais je voullais etre sur que ca le soit)



}
