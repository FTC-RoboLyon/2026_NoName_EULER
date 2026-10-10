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
import java.util.function.Supplier;

public class robotContainer extends Robot {
    private static VoltageSensor voltageSensor;
    private GamepadEx gamepad1, gamepad2;
    private Telemetry telemetry;


    private static DriveTrainSubsystem drivetrain;
    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;
    private static Camera camera;

    private Object alliance;

    public enum RobotMode {
        AUTO,
        TELEOP
    }
    private RobotMode robotMode = RobotMode.TELEOP;

    private int idRobot;

    public robotContainer (HardwareMap hmap, Telemetry telemetry, RobotMode robotMode1){

        robotMode = robotMode1;

        this.telemetry = telemetry;

        drivetrain = new DriveTrainSubsystem(hmap, this);
        shooter = new ShooterSubsystem(hmap, "Shooter", "Hood", this);
        intake = new IntakeSubsystem(hmap);
        camera = new Camera(hmap);

        drivetrain.setDriveMode(robotMode == RobotMode.TELEOP ? DriveTrainSubsystem.DriveMode.FIELD_CENTRIC : DriveTrainSubsystem.DriveMode.GO_TO_POS);

        voltageSensor = hmap.get(VoltageSensor.class, "Control Hub");
        setBulkReading(hmap, LynxModule.BulkCachingMode.AUTO);

        alliance = blackboard.get(ALLIANCE_KEY);

        camera.setTargetID(alliance == "red" ? 24 : 20);
    }

    public void bindCommands(Gamepad gamepad1, Gamepad gamepad2){

        DoubleSupplier forward = ()-> gamepad1.left_stick_x;
        DoubleSupplier strafe = ()-> gamepad1.left_stick_y;
        DoubleSupplier turn = ()-> gamepad1.right_stick_x;

        Supplier<Float> intakeBalls = ()-> gamepad1.right_trigger;
        Supplier<Float> ejectBalls = ()-> gamepad1.left_trigger;

        drivetrain.setSupplier(forward, strafe, turn);
        intake.setSuppliers(intakeBalls, ejectBalls);

        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);

        //TODO bind all commands to button here
    }

    public Telemetry getTelemetry(){return telemetry;}//Sympa mais dcp par contr la telemetry tu l'update ou ?
    public double getCameraBearing(){
        return camera.getBearing();
    }
    public double getCameraDistanceToGoal(){
        return camera.getDistanceMeters();
    }

    public static double getVoltage(){return voltageSensor.getVoltage();}

    public DriveTrainSubsystem getDriveTrain(){return drivetrain;}
    public ShooterSubsystem getShooter(){return shooter;}

    public IntakeSubsystem getIntake(){return intake;}

    public Camera getCamera(){return camera;}

}
