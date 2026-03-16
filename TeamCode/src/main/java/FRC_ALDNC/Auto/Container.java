package FRC_ALDNC.Auto;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;
import FRC_ALDNC.Auto.Subsystems.ShooterSubsystem;
import FRC_ALDNC.Auto.Subsystems.intakeSubsystem;

public class Container{
    public double x, y , xDepart, yDepart, angleDepart;
    HardwareMap hmap;
    Telemetry telemetry;
    public static DriveSubsystem driveSubsystem;
    intakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public Container(HardwareMap hamp, Telemetry telemetry, double xDepart, double yDepart, double angleDepart){
        this.angleDepart = angleDepart;
        this.yDepart = yDepart;
        this.xDepart = xDepart;
        this.telemetry = telemetry;
        this.hmap = hamp;
        intakeSubsystem = new intakeSubsystem(hmap, telemetry);
        driveSubsystem = new DriveSubsystem(hmap, telemetry, xDepart, yDepart, angleDepart);
        shooterSubsystem = new ShooterSubsystem(hmap);

    }
}
