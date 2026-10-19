package org.firstinspires.ftc.teamcode.CommandBase;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandBase.Command.SetShooterControlMode;
import org.firstinspires.ftc.teamcode.CommandBase.Command.SetShooterWantedState;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Camera.Camera;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Drivetrain.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.control.ControlMode;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class robotContainer extends Robot {
    private static VoltageSensor voltageSensor;
    private GamepadEx gamepadEx1, gamepadEx2;
    private Telemetry telemetry;


    private static DriveTrainSubsystem drivetrain;
    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;
    private static Camera camera;

    private String alliance = "red";

    public enum RobotMode {
        AUTO,
        TELEOP
    }
    private RobotMode robotMode = RobotMode.TELEOP;

    private int TargetAprilTagId;


    public robotContainer (HardwareMap hmap, Telemetry telemetry, RobotMode robotMode1){

        robotMode = robotMode1;

        this.telemetry = telemetry;

        drivetrain = new DriveTrainSubsystem(hmap, this);
        shooter = new ShooterSubsystem(hmap, "Shooter", "Hood", this);
        intake = new IntakeSubsystem(hmap,this, "Intake Motor");
        camera = new Camera(hmap);

        drivetrain.setDriveMode(robotMode == RobotMode.TELEOP ? DriveTrainSubsystem.DriveMode.FIELD_CENTRIC : DriveTrainSubsystem.DriveMode.GO_TO_POS);
        intake.setControlMode(robotMode == RobotMode.TELEOP ? ControlMode.MANUAL_DUTY_CYCLE : ControlMode.DUTY_CYCLE);

        voltageSensor = hmap.get(VoltageSensor.class, "Control Hub");
        setBulkReading(hmap, LynxModule.BulkCachingMode.AUTO);

        camera.setTargetID(alliance == "red" ? 24 : 20);
    }

    public void bindCommands(Gamepad gamepad1, Gamepad gamepad2){

        DoubleSupplier forward = ()-> gamepad1.left_stick_x;
        DoubleSupplier strafe = ()-> gamepad1.left_stick_y;
        DoubleSupplier turn = ()-> gamepad1.right_stick_x;

        Supplier<Float> intakeBalls = ()-> gamepad1.right_trigger;
        Supplier<Float> ejectBalls = ()-> gamepad1.left_trigger;

        BooleanSupplier plusSpeedSupplier = ()-> gamepad1.dpad_up;
        BooleanSupplier minusSpeedSupplier = ()-> gamepad1.dpad_down;
        BooleanSupplier increaseIncrementationSupplier = ()-> gamepad1.dpad_right;
        BooleanSupplier decreaseIncrementationSupplier = ()-> gamepad1.dpad_left;

        shooter.defineSupplier(
                plusSpeedSupplier,
                minusSpeedSupplier,
                increaseIncrementationSupplier,
                decreaseIncrementationSupplier);

        drivetrain.setSupplier(forward, strafe, turn);
        intake.setSuppliers(intakeBalls, ejectBalls);

        this.gamepadEx1 = new GamepadEx(gamepad1);
        this.gamepadEx2 = new GamepadEx(gamepad2);


        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetShooterWantedState(shooter, ShooterSubsystem.WantedState.SHOOT_AUTO));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SetShooterWantedState(shooter, ShooterSubsystem.WantedState.SHOOT_NEAR))
                .whenReleased(new SetShooterWantedState(shooter, ShooterSubsystem.WantedState.STAND_BY));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SetShooterWantedState(shooter, ShooterSubsystem.WantedState.SHOOT_MID))
                .whenReleased(new SetShooterWantedState(shooter, ShooterSubsystem.WantedState.STAND_BY));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SetShooterWantedState(shooter, ShooterSubsystem.WantedState.SHOOT_FAR))
                .whenReleased(new SetShooterWantedState(shooter, ShooterSubsystem.WantedState.STAND_BY));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SetShooterControlMode(shooter, ControlMode.MANUAL_VELOCITY));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SetShooterControlMode(shooter, ControlMode.VELOCITY_AUTO_PIDF));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(new GamepadButton(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(new SetShooterControlMode(shooter, ControlMode.DISABLED));

        //TODO bind all commands to button here
    }

    public Object getAlliance(){return alliance;}
    public Telemetry getTelemetry(){return telemetry;}
    public static double getVoltage(){return voltageSensor.getVoltage();}

    public DriveTrainSubsystem getDriveTrain(){return drivetrain;}
    public ShooterSubsystem getShooter(){return shooter;}

    public IntakeSubsystem getIntake(){return intake;}

    public Camera getCamera(){return camera;}

}
