package ALDNC_organe;
import static org.firstinspires.ftc.teamcode.Constant.COMPTEUR_BALLE;
import static org.firstinspires.ftc.teamcode.Constant.FEEDER;
import static org.firstinspires.ftc.teamcode.Constant.INTAKE;
import static org.firstinspires.ftc.teamcode.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Constant.SHOOTER;
import static org.firstinspires.ftc.teamcode.Constant.VISEUR;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "ALDNC_brain", group = "Euler")
public class ALDNC_brain extends LinearOpMode{

    private IMU imu;
    private VoltageSensor ControlHub_VoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        imu = hardwareMap.get(IMU.class, "imu");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        //DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);
        TouchSensor compteurBalle = hardwareMap.get(TouchSensor.class, COMPTEUR_BALLE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ALDNC_organe.shooter Shooter = new shooter(shooter);
        feeder_v1 Feeder = new feeder_v1(feeder);
        jambes Chassis = new jambes(left_motor, right_motor);
        bouche_intake Intake = new bouche_intake(intake);
        ALDNC_organe.viseur Volet = new viseur(viseur);

        double posviseur = 0.6;
        double posviseur_bank = 0.6;
        double Power_bank = 0.5;
        double posviseur_far = 0.37;
        double Power_far = 1;
        double PowerShooter = Power_far;
        double robotOrienDegrees;
        double real_velo = 0;
        float forward;
        float turn;
        double seuil_shootter = 12;
        double voltage = ControlHub_VoltageSensor.getVoltage();
        feeder.setPosition(0);
        PowerShooter = (PowerShooter * seuil_shootter) / voltage;
        Power_bank = (Power_bank * seuil_shootter) / voltage;
        Power_far = (Power_far * seuil_shootter) / voltage;
        boolean appuyage = compteurBalle.isPressed();
        boolean isShooting = false;
        int nbeBallesIn = 3;
        int vIntake = 0;
        boolean isFeeding = false;

        waitForStart();
        while (opModeIsActive()) {
            // Se déplacer
            turn = gamepad1.right_stick_x;
            forward = -gamepad1.left_stick_y;
            float valueLeftMotor = forward + turn;
            float valueRightMotor = forward - turn;
            /*if (!gamepad1.a) {
                valueLeftMotor /= 2;
                valueRightMotor /= 2;
            }else if (gamepad1.a){
                valueLeftMotor = valueLeftMotor /1;
                valueRightMotor = valueRightMotor /1;
            }*/
            Chassis.drive(valueLeftMotor, valueRightMotor);

            //intake
            Intake.intake(gamepad1.left_bumper,
                    gamepad1.left_trigger);

            nbeBallesIn = Intake.nbeBalles(appuyage, nbeBallesIn, vIntake);


            if (gamepad1.bWasPressed()){
                PowerShooter = Power_bank;
            } else if (gamepad1.yWasPressed()) {
                PowerShooter = Power_far;
            }


            if (gamepad1.rightBumperWasPressed()){
                isShooting = !isShooting;
            }

            if (feeder.getPosition() == 0.2){
                isFeeding = true;
            } else {
                isFeeding = false;
            }


            Shooter.shooter(PowerShooter,
                    gamepad1.rightBumperWasReleased(),
                    gamepad1.right_trigger,
                    isShooting);
            nbeBallesIn = Shooter.compteurBalles(nbeBallesIn, isShooting, isFeeding);



            Volet.viseur(gamepad1.a,
                    gamepad1.b,
                    gamepad1.y,
                    gamepad1.dpadRightWasPressed(),
                    gamepad1.dpadLeftWasPressed(),
                    posviseur,
                    posviseur_far,
                    posviseur_bank);



            //feeder
            Feeder.feeder(gamepad2.xWasPressed(), gamepad2.xWasReleased());

            real_velo = ((DcMotorEx) shooter).getVelocity();
            appuyage = compteurBalle.isPressed();
            telemetry.addData("Velocité programmé Shooter =", PowerShooter);
            telemetry.addData("Vrai vélocité Shooter =", real_velo);
            telemetry.addData("Position Viseur ", viseur.getPosition());
            telemetry.addData("Distance", compteurBalle.isPressed());
            telemetry.addData("is intaking", appuyage);
            telemetry.addData("Nbe Balles Inside Bot = ", nbeBallesIn);
            if (isShooting){
                telemetry.addLine("Shooter allumé");
            }else if(!isShooting){
                telemetry.addLine("Shooter éteint");
            }
            telemetry.update();
        }



    }
}
