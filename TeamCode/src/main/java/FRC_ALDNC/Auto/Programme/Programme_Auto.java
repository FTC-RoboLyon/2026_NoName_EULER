package FRC_ALDNC.Auto.Programme;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import FRC_ALDNC.Auto.Container;


@TeleOp(name = "AutoLyon", group = "FRC_ALDNC")
public class Programme_Auto extends CommandOpMode {

    Container container;
    private double xDepart = 28, yDepart = 39;

    @Override
    public void initialize() {

        container = new Container(hardwareMap, telemetry, gamepad1, xDepart, yDepart);

    }
}
