package ALDNC_organe;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static ALDNC_organe.Constant.COMPTEUR_BALLE;
import static ALDNC_organe.Constant.INTAKE;
import static ALDNC_organe.Constant.VISEUR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Webcam_aldnc_yeux.vision_opencv;
import FRC_ALDNC.SubSystem.Apriltag_reader;

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
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        VoltageSensor controlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);

        IMU.Parameters imu_parameters;
        imu_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(0, 0, 90)));
        imu.initialize(imu_parameters);
        imu.resetYaw();



        vision = new vision_opencv(hardwareMap);
        Apriltag_reader aprilJoke = new Apriltag_reader(hardwareMap);
        shooter Shooter = new shooter(hardwareMap);
        feeder_v1 Feeder = new feeder_v1(hardwareMap);
        jambes Chassis = new jambes(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER);
        bouche_intake Intake = new bouche_intake(hardwareMap);
        viseur Volet = new viseur(hardwareMap);

        double robotOrienDegrees = 0;
        final ElapsedTime timer = new ElapsedTime();
        Volet.setPosBank();


        while (opModeInInit()){
            selecAuto(selectauto, gamepad2.leftBumperWasPressed());
            telemetry.addData("Mode", selectauto);
        }
        waitForStart();

        while (!Shooter.isAtgoodspeed()) {
            Shooter.setVelo_bank();
        }

        for (int i = 0; i <= 3; i++){
            while (!Shooter.HaveShoot()){
                Feeder.setPosFeed();
            }
            timer.reset();
            while (timer.milliseconds()<1000){
                Feeder.setPosRepos();
                Intake.intake_simple();
            }

        }
        while (timer.milliseconds() < 500){
            Chassis.backward();
        }
        timer.reset();
        while (timer.milliseconds()<300){
            robotOrienDegrees = imu.getRobotYawPitchRollAngles().getYaw();

            if (selectauto == alliance.AutoRed){
                Chassis.turn_left();
            }
            else {
                Chassis.turn_right();
            }
        }
        timer.reset();
        while (timer.milliseconds() < 1000){
            Chassis.backward();
        }
        


    }
    private void selecAuto (alliance selectauto, boolean change){
        if (change) {
            if (selectauto == alliance.AutoBlue) {
                selectauto = alliance.AutoRed;
            }
            else if (selectauto == alliance.AutoRed) {
                selectauto = alliance.AutoBlue;
            }
        }
    }

}
