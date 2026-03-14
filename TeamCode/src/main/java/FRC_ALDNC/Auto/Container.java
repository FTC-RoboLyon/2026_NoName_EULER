package FRC_ALDNC.Auto;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Command.DriveCommand;
import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;
import FRC_ALDNC.Auto.Subsystems.intakeSubsystem;
import FRC_ALDNC.Auto.Command.intakeCommand;

public class Container{
    public double x, y;
    Gamepad manette;
    HardwareMap hmap;
    Telemetry telemetry;
    DriveSubsystem driveSubsystem;
    intakeSubsystem intakeSubsystem;
    public Container(HardwareMap hamp, Telemetry telemetry, Gamepad gamepad1){
        this.telemetry = telemetry;
        this.hmap = hamp;
        manette = gamepad1;
        driveSubsystem = new DriveSubsystem(hmap, telemetry);
        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, manette, telemetry, DriveCommand.driveMode.GoAngle, 0.0,0.0));
        intakeSubsystem = new intakeSubsystem(hamp, manette);
    }
}
