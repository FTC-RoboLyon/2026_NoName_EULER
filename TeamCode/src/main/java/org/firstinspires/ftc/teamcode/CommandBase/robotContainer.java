package org.firstinspires.ftc.teamcode.CommandBase;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel.allianceShifter.ALLIANCE_KEY;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Camera;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.R;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;

public class robotContainer extends Robot {
    private HardwareMap hardwareMap; //t'as vrm besoin de stocker ça ?'
    private static VoltageSensor voltageSensor;
    private Gamepad gamepad1, gamepad2;
    private Telemetry telemetry;


    private static DriveTrainSubsystem driveTrain; //normalement drivretrain c un seul mot mais bon...
    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;
    private static Camera camera;

    private Object alliance;

    public enum Periode{ //What is this strange language I see here ... ;) (I think that he wanted to say Period)
        AUTO,
        TELEOP
    }
    private Periode periode = Periode.TELEOP;

    public robotContainer (HardwareMap hmap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, Periode periode1){

        periode = periode1;

        hardwareMap = hmap;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        DoubleSupplier forward = ()-> gamepad1.left_stick_x;
        DoubleSupplier strafe = ()-> gamepad1.left_stick_y;
        DoubleSupplier turn = ()-> gamepad1.right_stick_x;

        driveTrain = new DriveTrainSubsystem(hardwareMap, this, forward, strafe, turn);
        shooter = new ShooterSubsystem(hardwareMap, "Shooter", "Hood", this);
        intake = new IntakeSubsystem(hardwareMap, gamepad1);
        camera = new Camera(hardwareMap);

        driveTrain.setDriveMode(periode == Periode.TELEOP ? DriveTrainSubsystem.DriveMode.FIELD_CENTRIC : DriveTrainSubsystem.DriveMode.GO_TO_POS);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        setBulkReading(hardwareMap, LynxModule.BulkCachingMode.AUTO);

        alliance = blackboard.get(ALLIANCE_KEY);
        alliance = (String) alliance; //meme l'IDE te dit que cette ligne ne sert a rien donc peut etre se poser la question de son utilité
    }

    public Telemetry getTelemetry(){return telemetry;}//Sympa mais dcp par contr la telemetry tu l'update ou ?
    public double getCameraBearing(){
        return camera.getBearing(alliance == "red" ? 24 : 20);
    }
    public double getCameraDistanceToGoal(){
        return camera.getDistanceMeters(alliance == "red" ? 24 : 20);
    }
    public static double getVoltage(){return voltageSensor.getVoltage();}
}
