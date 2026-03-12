package FRC_ALDNC.Auto;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Command.DriveCommand;
import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class Container{
    public double x, y;
    Gamepad manette;
    HardwareMap hmap;
    Telemetry telemetry;
    DriveSubsystem driveSubsystem;
    public Container(HardwareMap hamp, Telemetry telemetry, Gamepad gamepad1){
        this.telemetry = telemetry;
        this.hmap = hamp;
        manette = gamepad1;
        driveSubsystem = new DriveSubsystem(hmap, telemetry);
        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, manette));
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
}
