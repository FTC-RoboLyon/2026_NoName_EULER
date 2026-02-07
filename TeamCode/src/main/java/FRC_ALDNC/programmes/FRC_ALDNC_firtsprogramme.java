package FRC_ALDNC.programmes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;

@TeleOp(name = "FRC_ALDNC_firtsprogramme", group = "FRC_style")
public class FRC_ALDNC_firtsprogramme extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive_Train chassis = new Drive_Train(hardwareMap);
        Shooter_Subsystem shooter = new Shooter_Subsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()){

        }
    }
}
