package FRC_ALDNC.Auto.Programme;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import FRC_ALDNC.Auto.Container;


@TeleOp(name = "AutoLyon", group = "FRC_ALDNC")
public class Programme_Auto extends CommandOpMode {

    Container container;

    @Override
    public void initialize() {

        container = new Container(hardwareMap, telemetry, gamepad1);

    }
}
