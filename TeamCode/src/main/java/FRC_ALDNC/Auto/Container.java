package FRC_ALDNC.Auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;

public class Container{
    public double x, y;
    HardwareMap hmap;
    Telemetry telemetry;
    public Container(HardwareMap hamp, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hmap = hamp;
        DriveSubsystem driveSubsystem = new DriveSubsystem(hmap, telemetry);
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
}
