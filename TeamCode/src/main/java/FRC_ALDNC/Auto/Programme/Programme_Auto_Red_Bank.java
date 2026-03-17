package FRC_ALDNC.Auto.Programme;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FRC_ALDNC.Auto.Command.DriveCommand;
import FRC_ALDNC.Auto.Command.ShooterCommand;
import FRC_ALDNC.Auto.Container;


@TeleOp(name = "AutoLyon", group = "FRC_ALDNC")
public class Programme_Auto_Red_Bank extends CommandOpMode {

    Container container;
    HardwareMap hmap;
    private double xDepart = 321.4, yDepart = 321.4, angleDepart = Math.PI/2;


    @Override
    public void initialize() {


        container = new Container(hardwareMap, telemetry, xDepart, yDepart, angleDepart);
        new SequentialCommandGroup(new DriveCommand(Container.driveSubsystem, telemetry, DriveCommand.driveMode.GoAngle, 360, 360 ),
                new DriveCommand(Container.driveSubsystem, telemetry, DriveCommand.driveMode.GoPos, 300, 300),
                new ParallelCommandGroup(new ShooterCommand(Container.shooterSubsystem, ShooterCommand.shooterState.Bank))).schedule();


    }
}
