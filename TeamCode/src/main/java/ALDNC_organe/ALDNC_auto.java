package ALDNC_organe;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    enum alliance {
        AutoRed,
        AutoBlue,

    }
    private alliance selectauto = alliance.AutoRed;

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        VoltageSensor controlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);

        IMU.Parameters imu_parameters;
        imu_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(0, 0, 90)));
        imu.initialize(imu_parameters);
        imu.resetYaw();



        vision = new vision_opencv(hardwareMap);
        Apriltag_reader aprilJoke = new Apriltag_reader();
        shooter Shooter = new shooter(shooter);
        feeder_v1 Feeder = new feeder_v1(feeder);
        jambes Chassis = new jambes(left_motor, right_motor);
        bouche_intake Intake = new bouche_intake(intake);
        viseur Volet = new viseur(viseur);

        double robotOrienDegrees = 0;
        boolean isshooting = true;
        boolean isFeeding = false;
        double posviseur = 0.6;
        double posviseur_bank = 0.6;
        double Power_bank = 0.5;
        double posviseur_far = 0.37;
        double Power_far = 1;
        double PowerShooter = Power_bank;
        boolean left_trigger = false;
        final ElapsedTime timer = new ElapsedTime();
        int nbBalle = 0;
        boolean b_wpr = false;
        boolean y_wpr = false;
        float leftTrigger = 0;
        Volet.viseur(false, true, false, false, false);
        while (opModeInInit()){
            selecAuto(selectauto, gamepad2.leftBumperWasPressed());
            telemetry.addData("Mode", selectauto);
        }
        waitForStart();
        timer.reset();
        while (timer.milliseconds() < 3000) {
            Shooter.Shooter(isshooting,
                    PowerShooter,
                    false);
        }

        for (int i = 0; i <= 3; i++){

            timer.reset();
            while (timer.milliseconds()<500){
                Feeder.FEEEder(true, false);
            }
            timer.reset();
            while (timer.milliseconds()<100){
                Feeder.FEEEder(false, true);

            }
        }
        timer.reset();
        while (timer.milliseconds()<300){
            robotOrienDegrees = imu.getRobotYawPitchRollAngles().getYaw();

            if (selectauto == alliance.AutoRed){
                Chassis.turn_antihoraire();
            }
            else {
                Chassis.turn_horaire();
            }
        }
        timer.reset();
        while (timer.milliseconds() < 500){
            Chassis.backward();
        }
        


    }
    public alliance selecAuto (alliance selectauto, boolean change){
        if (change) {
            if (selectauto == alliance.AutoBlue) {
                selectauto = alliance.AutoRed;
            }
            else if (selectauto == alliance.AutoRed) {
                selectauto = alliance.AutoBlue;
            }
        }
        return selectauto;
    }

}
