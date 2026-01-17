package ALDNC_organe;

import static ALDNC_organe.Constant.COMPTEUR_BALLE;
import static ALDNC_organe.Constant.FEEDER;
import static ALDNC_organe.Constant.INTAKE;
import static ALDNC_organe.Constant.LEFT_MOTOR;
import static ALDNC_organe.Constant.RIGHT_MOTOR;
import static ALDNC_organe.Constant.SHOOTER;
import static ALDNC_organe.Constant.VISEUR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import Webcam_aldnc_yeux.Apriltag_reader;
import Webcam_aldnc_yeux.Vrai_vision;
import Webcam_aldnc_yeux.vision_opencv;

@TeleOp(name = "ALDNC_brain", group = "Euler")
public class ALDNC_brain extends LinearOpMode {

    enum ModeRobot {Manuel, ChercheBalle, place_shoot}
    enum Pos_Balle {gauche, centre, droite, non_detected}

    public enum VisionMode {
        APRILTAG,
        OBJECT_TRACKING
    }
    enum alliance {
        Red,
        Blue,
        Teleop

    }
    private alliance selectauto = alliance.Teleop;
    private VisionMode visionMode = VisionMode.APRILTAG;

    static ModeRobot robot = ModeRobot.Manuel;
    static Pos_Balle balle = Pos_Balle.non_detected;

    Apriltag_reader aprilJoke = new Apriltag_reader();
    Vrai_vision objectProcessor = new Vrai_vision();




    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        VoltageSensor controlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);


        shooter Shooter = new shooter(shooter);
        feeder_v1 Feeder = new feeder_v1(feeder);
        jambes Chassis = new jambes(left_motor, right_motor);
        bouche_intake Intake = new bouche_intake(intake);
        viseur Volet = new viseur(viseur);
        Apriltag_reader aprilJoke = new Apriltag_reader();
        Vrai_vision objectProcessor = new Vrai_vision();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilJoke)
                .addProcessor(objectProcessor)
                .build();


        double Power_bank = 0.5;
        double Power_mid = 0.75;
        double Power_far = 1;
        double PowerShooter = Power_bank;
        double robotOrienDegrees;
        double real_velo = 0;
        float forward;
        float turn;
        double seuil_shootter = 12;
        double voltage = controlHub_VoltageSensor.getVoltage();
        feeder.setPosition(0);
        PowerShooter = (PowerShooter * seuil_shootter) / voltage;
        Power_bank = (Power_bank * seuil_shootter) / voltage;
        Power_far = (Power_far * seuil_shootter) / voltage;
        double distance = compteurBalle.getDistance(DistanceUnit.CM);
        boolean isIntaking = false;
        boolean isShooting = false;
        int nbeBallesIn = 0;
        int vIntake = 0;
        boolean isFeeding = false;
        int a = 0;
        int b = 0;
        boolean left_trigger = false;
        boolean leftrig;
        AprilTagDetection actual_april = null;



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            switch (visionMode) {
                case APRILTAG:
                    visionPortal.setProcessorEnabled(aprilJoke, true);
                    visionPortal.setProcessorEnabled(objectProcessor, false);
                    break;
                case OBJECT_TRACKING:
                    visionPortal.setProcessorEnabled(aprilJoke, false);
                    visionPortal.setProcessorEnabled(objectProcessor, true);
                    break;
            }

            actual_april = aprilJoke.getBestAprilTag(null);


            //camera
            /*if (vision.isObjectDetected()) {
                int x = vision.getObjectX();
                if (x < 213) {
                    telemetry.addLine("Objet à GAUCHE");
                    balle = Pos_Balle.gauche;
                } else if (x < 426) {
                    telemetry.addLine("Objet au CENTRE");
                    balle = Pos_Balle.centre;
                } else {
                    telemetry.addLine("Objet à DROITE");
                    balle = Pos_Balle.droite;
                }
            } else {
                telemetry.addLine("Aucun objet détecté");
            }
            /*
             */

            //actual_april = aprilJoke.getBestAprilTag(null);
            //isIntaking = distance < 25;
            //nbeBallesIn = Intake.nbeBalles(distance, nbeBallesIn);

            //switch_states(gamepad2.x, gamepad2.y, gamepad2.b);
            //switch (robot) {
            //case Manuel:


            // Se déplacer
            turn = gamepad1.right_stick_x;
            forward = -gamepad1.left_stick_y;
            float valueLeftMotor = forward + turn;
            float valueRightMotor = forward - turn;
            if(isShooting){
                valueRightMotor /= 2;
                valueLeftMotor /= 2;
            }
            Chassis.drive(valueLeftMotor, valueRightMotor);

            //intake
            Intake.intake(gamepad1.left_bumper,
                    gamepad1.left_trigger);



                    /*if (Shooter.getPower() <= PowerShooter + 0.05 && Shooter.getPower() >= PowerShooter - 0.05){
                        gamepad1.rumble(100);
                    }*/

            //shooting
            if(gamepad1.rightBumperWasPressed()){
                isShooting = !isShooting;
            }
            leftrig = gamepad1.right_trigger > 0.3;


            Shooter.shooter(isShooting,
                    PowerShooter,
                    leftrig);


            PowerShooter = Shooter.regleurPuissanceShooter(PowerShooter,
                    gamepad2.dpad_up,
                    gamepad2.dpad_down,
                    gamepad2.dpad_left,
                    gamepad2.dpad_right,
                    gamepad1.bWasPressed(),
                    gamepad1.yWasPressed(),
                    gamepad1.aWasPressed(),
                    Power_bank, Power_mid,
                    Power_far);


            //vising
            Volet.viseur(gamepad1.a,
                    gamepad1.b,
                    gamepad1.y,
                    gamepad1.dpadRightWasPressed(),
                    gamepad1.dpadLeftWasPressed());


            //feeder

            isFeeding = Feeder.feederPara(gamepad1.xWasPressed(), isFeeding, isShooting);
            Feeder.feed(isFeeding, isShooting);


                /*case ChercheBalle:
                    switch (balle) {
                        case gauche:
                            Chassis.turn_antihoraire();
                            break;
                        case droite:
                            Chassis.turn_horaire();
                            break;
                        case centre:
                            Chassis.forward();
                            Intake.intake_simple();
                            if (Intake.isvIntake()) {
                                Chassis.stop();
                                Intake.stop_intake();
                            }
                            break;
                        case non_detected:
                            robot = ModeRobot.Manuel;
                    }
                    break;*/


            real_velo = ((DcMotorEx) shooter).getVelocity();
            distance = compteurBalle.getDistance(DistanceUnit.CM);
            telemetry.addData("Velocité programmé Shooter =", PowerShooter);
            telemetry.addData("Vrai vélocité Shooter =", real_velo);
            telemetry.addData("Position Viseur ", viseur.getPosition());
            telemetry.addData("Distance", compteurBalle.getDistance(DistanceUnit.CM));
            telemetry.addData("is intaking", isIntaking);
            telemetry.addData("Nbe Balles Inside Bot = ", nbeBallesIn);
            telemetry.addData("distance", distance);
            //telemetry.addData("objet detécté",vision.isObjectDetected() );
            telemetry.addLine(isShooting ? "Shooter allumé" : "Shooter éteint");
            telemetry.addData("puissance", PowerShooter);
            telemetry.addData("posfeed", Feeder.getposition());

            if (actual_april != null) {
                telemetry.addData("ID de l'april", actual_april.id);
                telemetry.addData("décalage gauche-droite a l'april", actual_april.ftcPose.x);
                telemetry.addData("Hauteur a l'april", actual_april.ftcPose.y);
                telemetry.addData("distance a l'april", actual_april.ftcPose.z);
                telemetry.addData("orientation a l'april", actual_april.ftcPose.yaw);
            } else telemetry.addLine("aucun apriltag détécté");
            telemetry.update();
          }
        }


    }






    /*public void switch_states(boolean gmp2_x, boolean gmp2_y, boolean gmp2_b) {
        if (gmp2_x) {
            robot = ModeRobot.Manuel;
        } else if (gmp2_y) {
            robot = ModeRobot.ChercheBalle;
        } else if (gmp2_b) {
            robot = ModeRobot.place_shoot;
        }
    }*/





