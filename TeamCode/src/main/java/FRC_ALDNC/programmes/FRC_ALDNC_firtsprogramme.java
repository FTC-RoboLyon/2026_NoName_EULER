package FRC_ALDNC.programmes;

import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import FRC_ALDNC.ALDNC_container;

@TeleOp(name = "FRC_ALDNC_firtsprogramme", group = "FRC_style")
public class FRC_ALDNC_firtsprogramme extends CommandOpMode {
    ALDNC_container robot;
    GamepadEx gamepad0 = new GamepadEx(gamepad1);
    GamepadEx gamEpad2 = new GamepadEx(gamepad2);
    Button feeder_button = new GamepadButton(
            gamepad0, GamepadKeys.Button.X
    );
    Button shoot_bank_button = new GamepadButton(
            gamepad0, GamepadKeys.Button.B
    );
    Button shoot_mid_button = new GamepadButton(
            gamepad0, GamepadKeys.Button.Y
    );
    Button shoot_far_button = new GamepadButton(
            gamepad0, GamepadKeys.Button.A
    );
    Button Intake_button = new GamepadButton(
            gamepad0, GamepadKeys.Button.LEFT_BUMPER
    );


    @Override
    public void initialize() {
        robot = new ALDNC_container(hardwareMap, ALDNC_container.RobotMode.TELEOP_RED, gamepad0, telemetry);
        robot.Configure_Binding(feeder_button,shoot_bank_button, shoot_mid_button, shoot_far_button, Intake_button);

    }

}
