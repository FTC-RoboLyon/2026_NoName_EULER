package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class robotContainer extends Robot {
    private HardwareMap hardwareMap;
    private static VoltageSensor voltageSensor;
    private Gamepad gamepad1, gamepad2;
    private Telemetry telemetry;


    private static DriveTrainSubsystem driveTrain;
    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;

    public robotContainer (HardwareMap hmap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){

        hardwareMap = hmap;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        DoubleSupplier forward = ()-> gamepad1.left_stick_x;
        DoubleSupplier strafe = ()-> gamepad1.left_stick_y;
        DoubleSupplier turn = ()-> gamepad1.right_stick_x;

        driveTrain = new DriveTrainSubsystem(hardwareMap, forward, strafe, turn);
        shooter = new ShooterSubsystem(hardwareMap, "Shooter", "Hood", this);
        intake = new IntakeSubsystem();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);
    }
    public Telemetry getTelemetry(){return telemetry;}
    public static double getVoltage(){return voltageSensor.getVoltage();}
}
