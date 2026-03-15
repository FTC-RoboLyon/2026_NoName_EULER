package FRC_ALDNC.Auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Command.DriveCommand;
import FRC_ALDNC.Auto.Command.ShooterCommand;
import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;
import FRC_ALDNC.Auto.Subsystems.ShooterSubsystem;
import FRC_ALDNC.Auto.Subsystems.intakeSubsystem;
import FRC_ALDNC.Auto.Command.intakeCommand;

public class Container{
    public double x, y , xDepart, yDepart;
    Gamepad manette;
    HardwareMap hmap;
    Telemetry telemetry;
    DriveSubsystem driveSubsystem;
    intakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    public Container(HardwareMap hamp, Telemetry telemetry, Gamepad gamepad1, double xDepart, double yDepart){
        this.yDepart = yDepart;
        this.xDepart = xDepart;
        this.telemetry = telemetry;
        this.hmap = hamp;
        manette = gamepad1;
        driveSubsystem = new DriveSubsystem(hmap, telemetry, xDepart, yDepart);
        new SequentialCommandGroup(new DriveCommand(driveSubsystem, manette, telemetry, DriveCommand.driveMode.GoAngle, 180.0, 180.0),
                new DriveCommand(driveSubsystem, manette, telemetry, DriveCommand.driveMode.GoPos, 180.0, 180.0),
                new DriveCommand(driveSubsystem, manette, telemetry, DriveCommand.driveMode.GoAngle, 28.0, 39.0),
                new DriveCommand(driveSubsystem, manette, telemetry, DriveCommand.driveMode.GoPos, 28.0, 39.0),
                new DriveCommand(driveSubsystem, manette, telemetry, DriveCommand.driveMode.GoAngle, 0.0,0.0)).schedule();

    }
}
