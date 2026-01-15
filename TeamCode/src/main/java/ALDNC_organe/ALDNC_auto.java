package ALDNC_organe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static ALDNC_organe.Constant.COMPTEUR_BALLE;
import static ALDNC_organe.Constant.FEEDER;
import static ALDNC_organe.Constant.INTAKE;
import static ALDNC_organe.Constant.LEFT_MOTOR;
import static ALDNC_organe.Constant.RIGHT_MOTOR;
import static ALDNC_organe.Constant.SHOOTER;
import static ALDNC_organe.Constant.VISEUR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Webcam_aldnc_yeux.vision_opencv;
import Webcam_aldnc_yeux.Apriltag_reader;

@Autonomous (name = "ALDNC_auto", group = "Euler")
public class ALDNC_auto extends LinearOpMode {

    enum Pos_Balle {gauche, centre, droite, non_detected}
    vision_opencv vision;
    private IMU imu;
    private VoltageSensor ControlHub_VoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        imu = hardwareMap.get(IMU.class, "imu");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);


        vision = new vision_opencv(hardwareMap);
        Apriltag_reader aprilJoke = new Apriltag_reader();
        shooter Shooter = new shooter(shooter);
        feeder_v1 Feeder = new feeder_v1(feeder);
        jambes Chassis = new jambes(left_motor, right_motor);
        bouche_intake Intake = new bouche_intake(intake);
        viseur Volet = new viseur(viseur);

        boolean isshooting = true;
        boolean isFeeding = false;
        double posviseur = 0.6;
        double posviseur_bank = 0.6;
        double Power_bank = 0.5;
        double posviseur_far = 0.37;
        double Power_far = 1;
        double PowerShooter = Power_far;
        boolean left_trigger = false;
        final ElapsedTime timer = new ElapsedTime();
        int nbBalle = 0;
        boolean b_wpr = false;
        boolean y_wpr = false;
        float leftTrigger = 0;



        waitForStart();
        timer.reset();
        while (timer.milliseconds() < 5000) {
            Shooter.shooter(isshooting,
                    PowerShooter,
                    leftTrigger,
                    b_wpr,
                    y_wpr,
                    Power_bank,
                    Power_far);
            isFeeding = Feeder.feederPara(true, isFeeding, isshooting);
            Feeder.feeder(isFeeding, isshooting);
        }
        while (opModeIsActive()){

        }


    }

}
