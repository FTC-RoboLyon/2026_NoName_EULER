package ALDNC_organe;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ALDNC_brain", group = "Euler")
@Disabled
public class PIDF_tuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        shooter shooter = new shooter(hardwareMap);
        jambes Chassis = new jambes(hardwareMap);
        viseur Volet = new viseur(hardwareMap);
        feeder_v1 Feeder = new feeder_v1(hardwareMap);
        bouche_intake Intake = new bouche_intake(hardwareMap);

        float forward;
        float turn;

        waitForStart();
        while (opModeIsActive()){
            turn = gamepad1.right_stick_x;
            forward = -gamepad1.left_stick_y;
            double valueLeftMotor    = Range.clip(forward + turn, -1.0, 1.0) ;
            double valueRightMotor   = Range.clip(forward - turn, -1.0, 1.0) ;
            if(shooter.isShooting){
                valueRightMotor /= 2;
                valueLeftMotor /= 2;
            }
            Chassis.drive(valueLeftMotor, valueRightMotor);
            shooter.regleurVeloShooteur(
                    gamepad1, gamepad2);

            Volet.viseur(gamepad1, gamepad2);

            Feeder.Feeder(gamepad1);

            Intake.intake(gamepad1.left_bumper,
                    gamepad1.left_trigger);


        }

    }
}
