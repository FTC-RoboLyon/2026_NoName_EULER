package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics.Pose2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

public class Path extends SequentialCommandGroup {

    DriveTrainSubsystem driveTrainSubsystem;

    Pose2d[] points;

    public Path (DriveTrainSubsystem drivetrain, Pose2d... points){
        driveTrainSubsystem = drivetrain;
        driveTrainSubsystem.setDriveMode(DriveTrainSubsystem.DriveMode.GO_TO_POS);
        this.points = points;
        addRequirements(driveTrainSubsystem);

        ArrayList<GoToPosCommand> steps = new ArrayList<>();

        for (Pose2d point : points){
            steps.add(new GoToPosCommand(driveTrainSubsystem, point));
        }

        GoToPosCommand[] commands = steps.toArray(new GoToPosCommand[0]);

        addCommands(commands);
    }

}
