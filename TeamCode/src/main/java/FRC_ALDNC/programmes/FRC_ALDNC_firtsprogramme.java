package FRC_ALDNC.programmes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import FRC_ALDNC.SubSystem.Drive_Train;
import FRC_ALDNC.SubSystem.Shooter_Subsystem;
import Webcam_aldnc_yeux.Apriltag_reader;
import packageClermont.organe.joySticks.joyStickY;
import packageClermont.organe.joySticks.joystickX;

@TeleOp(name = "FRC_ALDNC_firtsprogramme", group = "FRC_style")
public class FRC_ALDNC_firtsprogramme extends LinearOpMode {
    double rightX;
    double leftY;
    double x1 = 0;
    double y1 = 0;
    double value_jambeDroite;
    double value_jambeGauche;
    @Override
    public void runOpMode() throws InterruptedException {
        Drive_Train chassis = new Drive_Train(hardwareMap);
        Shooter_Subsystem shooter = new Shooter_Subsystem(hardwareMap);
        joyStickY joyStickY = new joyStickY(telemetry);
        joystickX joystickX = new joystickX(telemetry);
        Apriltag_reader apriljoke = new Apriltag_reader(hardwareMap);


        waitForStart();
        while (opModeIsActive()){
            rightX = joystickX.joyStickXPara(gamepad1.right_stick_x, x1, gamepad1.left_stick_y, y1);
            leftY = joyStickY.joyStickYPara(gamepad1.right_stick_x, x1, gamepad1.left_stick_y, y1);
            x1 = rightX;
            y1 = leftY;

            value_jambeDroite = rightX - leftY;
            value_jambeGauche = rightX + leftY;
            chassis.drive(value_jambeGauche, value_jambeDroite);
            chassis.periodic();

            apriljoke.updtade();
            if (gamepad1.left_stick_button){
                chassis.align_rotation(0, apriljoke.getAprilTagById(24).ftcPose.bearing);
            }

        }
    }
}
