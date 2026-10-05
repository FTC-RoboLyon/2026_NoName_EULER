package org.firstinspires.ftc.teamcode.CommandBase.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Camera;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;

public class DriveWhileHeadingCommand extends CommandBase {

    DriveTrainSubsystem driveTrainSubsystem;
    robotContainer robot;
    public DriveWhileHeadingCommand(DriveTrainSubsystem driveTrain, robotContainer robot){
        driveTrainSubsystem = driveTrain;
        this.robot = robot;
        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize(){
        driveTrainSubsystem.setDriveMode(DriveTrainSubsystem.DriveMode.DRIVE_AND_HEAD_TO_TARGET);
    }

    @Override
    public void execute(){
        if (robot.getCameraBearing() != -7.0)
            driveTrainSubsystem.setHeadingTarget(driveTrainSubsystem.getRobotHeading() - robot.getCameraBearing());
        //Et pourquoi la base n'irait-elle pas chercher l'info de la cam elle meme prc que la dcp tant qu'elle est pas alignee elle est occupee par une commande
        // donc on peut pas lui changer de consigne et en plus des qu'elle est algnee la commande disable la base qui n'est donc plus utilisable apres
        else {
            robot.getTelemetry().addLine("Camera isn't seeing a target");
            driveTrainSubsystem.setHeadingTarget(driveTrainSubsystem.getRobotHeading());
        }
    }

    @Override
    public void end(boolean interupted){
        if (interupted)
            driveTrainSubsystem.setDriveMode(DriveTrainSubsystem.DriveMode.DISABLE);
        //est tu sur de bien comprendre a quoi correspondent la fonction end et son parametre interrupted
    }

}
