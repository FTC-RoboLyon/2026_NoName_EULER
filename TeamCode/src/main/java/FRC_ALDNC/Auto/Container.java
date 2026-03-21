package FRC_ALDNC.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

import FRC_ALDNC.Auto.Subsystems.DriveSubsystem;
import FRC_ALDNC.SubSystem.NavXSubsystem;
import FRC_ALDNC.Auto.Subsystems.ShooterSubsystem;
import FRC_ALDNC.Auto.Subsystems.IntakeSubsystem;
import FRC_ALDNC.Auto.Subsystems.FeederSubsystem;
import FRC_ALDNC.SubSystem.Camera_subsystem;

public class Container{
    public double x, y , xDepart, yDepart, angleDepart;
    HardwareMap hmap;
    Telemetry telemetry;
    public static NavXSubsystem navx;
    public static DriveSubsystem driveSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static FeederSubsystem feeder_subsystem;
    public static Camera_subsystem cameraSubsystem;
    public Container(HardwareMap hamp, Telemetry telemetry, double xDepart, double yDepart, double angleDepart){
        this.angleDepart = angleDepart;
        this.yDepart = yDepart;
        this.xDepart = xDepart;
        this.telemetry = telemetry;
        this.hmap = hamp;
        navx = new NavXSubsystem(hmap);
        cameraSubsystem = new Camera_subsystem(hamp, 20, telemetry);
        feeder_subsystem = new FeederSubsystem(hmap);
        intakeSubsystem = new IntakeSubsystem(hmap, telemetry);
        driveSubsystem = new DriveSubsystem(navx, hmap, telemetry, xDepart, yDepart, angleDepart);
        shooterSubsystem = new ShooterSubsystem(hmap);


    }
}
