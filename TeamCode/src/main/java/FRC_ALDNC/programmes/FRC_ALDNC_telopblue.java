package FRC_ALDNC.programmes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.BooleanSupplier;

import FRC_ALDNC.ALDNC_container;

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
    BooleanSupplier lefT_triger;


    @Override
    public void initialize() {
        gamepad0 = new GamepadEx(gamepad1);
        gamEpad2 = new GamepadEx(gamepad2);
        alignageButton = new GamepadButton(
                gamEpad2, GamepadKeys.Button.DPAD_UP
        );
        lefT_triger = () -> gamepad1.left_trigger > 0.3;
        feeder_button = new GamepadButton(
                gamepad0, GamepadKeys.Button.X
        );
        shoot_bank_button = new GamepadButton(
                gamepad0, GamepadKeys.Button.B
        );
        shoot_mid_button = new GamepadButton(
                gamepad0, GamepadKeys.Button.Y
        );
        shoot_far_button = new GamepadButton(
                gamepad0, GamepadKeys.Button.A
        );
        aspirer_button = new GamepadButton(
                gamepad0, GamepadKeys.Button.RIGHT_BUMPER
        );
        Intake_button = new GamepadButton(
                gamepad0, GamepadKeys.Button.LEFT_BUMPER
        );
        Eject_button = new Trigger(lefT_triger);

        robot = new ALDNC_container(hardwareMap, ALDNC_container.RobotMode.TELEOP_BLUE, gamepad0, telemetry);
        robot.Configure_Binding(feeder_button,shoot_bank_button, shoot_mid_button, shoot_far_button, aspirer_button,Intake_button , Eject_button, alignageButton);
        robot.telemetry();

    }
}
