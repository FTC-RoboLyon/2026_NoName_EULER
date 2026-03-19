package FRC_ALDNC.programmes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.BooleanSupplier;

import FRC_ALDNC.ALDNC_container;
@TeleOp(name = "FRC ALDNC teleop BLUE", group = "FRC_style")
public class FRC_ALDNC_telopblue extends CommandOpMode {
    ALDNC_container robot;
    GamepadEx gamepad0;
    GamepadEx gamEpad2;
    Button feeder_button;
    Button shoot_bank_button;
    Button shoot_mid_button ;
    Button shoot_far_button;
    Button aspirer_button;
    Button Intake_button ;
    Trigger Eject_button ;
    Button alignageButton;
    Button reglage_shooter;
    BooleanSupplier lefT_triger;
    BooleanSupplier right_trigger;

    BooleanSupplier slefT_triger;
    BooleanSupplier sright_trigger;

    Trigger aspirer;
    Button plus_velo;
    Button minus_velo;
    Button splus_velo;
    Button sminus_velo;

    Button plus_viseur;
    Button minus_viseur;
    Trigger splus_viseur;
    Trigger sminus_viseur;


    @Override
    public void initialize() {
        gamepad0 = new GamepadEx(gamepad1);
        gamEpad2 = new GamepadEx(gamepad2);
        lefT_triger = () -> gamepad1.left_trigger > 0.3;
        right_trigger = () -> gamepad1.back;

        slefT_triger = () -> gamepad2.left_trigger > 0.3;
        sright_trigger = () -> gamepad2.right_trigger > 0.3;

        alignageButton = new GamepadButton(
                gamepad0, GamepadKeys.Button.DPAD_UP
        );
        feeder_button = new GamepadButton(
                gamEpad2, GamepadKeys.Button.X
        );
        shoot_bank_button = new GamepadButton(
                gamEpad2, GamepadKeys.Button.B
        );
        shoot_mid_button = new GamepadButton(
                gamEpad2, GamepadKeys.Button.Y
        );
        shoot_far_button = new GamepadButton(
                gamEpad2, GamepadKeys.Button.A
        );
        aspirer_button = new GamepadButton(
                gamEpad2, GamepadKeys.Button.RIGHT_BUMPER
        );
        Intake_button = new GamepadButton(
                gamepad0, GamepadKeys.Button.LEFT_BUMPER
        );
        reglage_shooter = new GamepadButton(
                gamEpad2, GamepadKeys.Button.LEFT_BUMPER
        );
        plus_velo = new GamepadButton(
                gamEpad2, GamepadKeys.Button.DPAD_UP
        );
        splus_velo = new GamepadButton(
                gamEpad2, GamepadKeys.Button.DPAD_RIGHT
        );
        minus_velo = new GamepadButton(
                gamEpad2, GamepadKeys.Button.DPAD_DOWN
        );
        sminus_velo = new GamepadButton(
                gamEpad2, GamepadKeys.Button.DPAD_LEFT
        );
        minus_viseur = new GamepadButton(
                gamEpad2, GamepadKeys.Button.RIGHT_BUMPER
        );
        plus_viseur = new GamepadButton(
                gamEpad2, GamepadKeys.Button.LEFT_STICK_BUTTON
        );

        Eject_button = new Trigger(lefT_triger);
        aspirer = new Trigger(right_trigger);

        splus_viseur = new Trigger(slefT_triger);
        sminus_viseur = new Trigger(sright_trigger);

        robot = new ALDNC_container(hardwareMap, ALDNC_container.RobotMode.TELEOP_BLUE, gamepad0, telemetry);
        robot.Configure_Binding(feeder_button,
                shoot_bank_button,
                shoot_mid_button, shoot_far_button,
                aspirer_button,Intake_button ,
                Eject_button, alignageButton,
                reglage_shooter,
                aspirer,
                plus_viseur,
                splus_viseur,
                minus_viseur,
                sminus_viseur,
                plus_velo,
                splus_velo,
                minus_velo,
                sminus_velo);
        robot.telemetry();

    }
}
