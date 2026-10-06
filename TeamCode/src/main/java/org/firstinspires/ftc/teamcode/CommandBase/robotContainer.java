package org.firstinspires.ftc.teamcode.CommandBase;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.allianceShifter.ALLIANCE_KEY;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Camera;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class robotContainer extends Robot {
    private HardwareMap hardwareMap;
    private static VoltageSensor voltageSensor;
    private GamepadEx gamepad1, gamepad2;
    private Telemetry telemetry;


    private static DriveTrainSubsystem driveTrain;
    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;
    private static Camera camera;

    private Object alliance;

    public enum RobotMode {
        AUTO,
        TELEOP
    }
    private RobotMode robotMode = RobotMode.TELEOP;

    public robotContainer (HardwareMap hmap, Telemetry telemetry, RobotMode robotMode1){

        robotMode = robotMode1;

        hardwareMap = hmap;

        this.telemetry = telemetry;

        driveTrain = new DriveTrainSubsystem(hardwareMap, this);
        shooter = new ShooterSubsystem(hardwareMap, "Shooter", "Hood", this);
        intake = new IntakeSubsystem(hardwareMap);
        camera = new Camera(hardwareMap);

        driveTrain.setDriveMode(robotMode == RobotMode.TELEOP ? DriveTrainSubsystem.DriveMode.FIELD_CENTRIC : DriveTrainSubsystem.DriveMode.GO_TO_POS);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        alliance = blackboard.get(ALLIANCE_KEY);
        alliance = (String) alliance;
    }

    public void bindCommands(Gamepad gamepad1, Gamepad gamepad2){

        DoubleSupplier forward = ()-> gamepad1.left_stick_x;
        DoubleSupplier strafe = ()-> gamepad1.left_stick_y;
        DoubleSupplier turn = ()-> gamepad1.right_stick_x;

        driveTrain.setSupplier(forward, strafe, turn);
        intake.setGamepad(gamepad1);

        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);
    }

    public Telemetry getTelemetry(){return telemetry;}
    public double getCameraBearing(){
        return camera.getBearing(alliance == "red" ? 24 : 20);
    }
    public double getCameraDistanceToGoal(){
        return camera.getDistanceMeters(alliance == "red" ? 24 : 20);
    }

    public static double getVoltage(){return voltageSensor.getVoltage();}

    public DriveTrainSubsystem getDriveTrain(){return driveTrain;}
    public ShooterSubsystem getShooter(){return shooter;}

    public IntakeSubsystem getIntake(){return intake;}

    public Camera getCamera(){return camera;}

}
