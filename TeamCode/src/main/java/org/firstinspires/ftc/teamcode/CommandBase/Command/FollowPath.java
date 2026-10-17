package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics.Pose2d;

import java.util.ArrayList;

public class FollowPath extends SequentialCommandGroup {
    //nom pas tres clair, il ne permet pas de deviner ce qu'il fait on pourrait croire su'il stocke tout les infos d'un chemin plutot
    DriveTrainSubsystem driveTrainSubsystem;

    Pose2d[] points;

    public FollowPath(DriveTrainSubsystem drivetrain, Pose2d... points){
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
