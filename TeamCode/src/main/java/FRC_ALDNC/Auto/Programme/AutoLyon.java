package FRC_ALDNC.Auto.Programme;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FRC_ALDNC.Auto.Command.DriveCommand;
import FRC_ALDNC.Auto.Command.FeederCommand;
import FRC_ALDNC.Auto.Command.ShooterCommand;
import FRC_ALDNC.Auto.Container;
import FRC_ALDNC.commands.Let_a_ball_pass;


@TeleOp(name = "AutoLyon", group = "FRC_ALDNC")
public class AutoLyon extends CommandOpMode {

    Container container;
    HardwareMap hmap;
    private double xDepart = 325, yDepart = 25, angleDepart = Math.PI/2;


    @Override
    public void initialize() {


        container = new Container(hardwareMap, telemetry, xDepart, yDepart, angleDepart);
        Container.driveSubsystem.resetAngle();
        new SequentialCommandGroup(new DriveCommand(Container.driveSubsystem, telemetry, DriveCommand.driveMode.GoPos, 180, 180 ),
                new DriveCommand(Container.driveSubsystem, telemetry, DriveCommand.driveMode.GoAngle, 325, 25),
                new ParallelRaceGroup(new ShooterCommand(Container.shooterSubsystem, ShooterCommand.shooterState.Far),
                        new FeederCommand(Container.feeder_subsystem, telemetry, Container.shooterSubsystem))
        ).schedule();


    }
}
