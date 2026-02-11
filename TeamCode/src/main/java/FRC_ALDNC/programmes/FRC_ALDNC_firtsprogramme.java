package FRC_ALDNC.programmes;

import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import FRC_ALDNC.ALDNC_container;

@TeleOp(name = "FRC_ALDNC_firtsprogramme", group = "FRC_style")
public class FRC_ALDNC_firtsprogramme extends CommandOpMode {
    ALDNC_container robot;
    GamepadEx gamepad0 = new GamepadEx(gamepad1);
    GamepadEx gamEpad2 = new GamepadEx(gamepad2);

    @Override
    public void initialize() {
        robot = new ALDNC_container(hardwareMap, ALDNC_container.RobotMode.TELEOP_RED, gamepad1, telemetry);
    }

}
