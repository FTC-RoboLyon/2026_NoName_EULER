package ALDNC_organe;

import static ALDNC_organe.Constant.COMPTEUR_BALLE;
import static ALDNC_organe.Constant.INTAKE;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.util.Range;


import Webcam_aldnc_yeux.Apriltag_reader;
import Webcam_aldnc_yeux.Vrai_vision;

@TeleOp(name = "ALDNC_brain", group = "Euler")
public class ALDNC_brain extends LinearOpMode {

    enum ModeRobot {Manuel, ChercheBalle, place_shoot}
    enum Pos_Balle {gauche, centre, droite, non_detected}

    public enum VisionMode {
        APRILTAG,
        OBJECT_TRACKING
    }
    enum alliance {
        RedAuto,
        BlueAuto,
        RedTeleop,
        BlueTeleop

    }
    private final alliance selectauto = alliance.RedTeleop;
    private final VisionMode visionMode = VisionMode.APRILTAG;

    static ModeRobot robot = ModeRobot.Manuel;
    static Pos_Balle balle = Pos_Balle.non_detected;

    Apriltag_reader aprilJoke = new Apriltag_reader();
    Vrai_vision objectProcessor = new Vrai_vision();




    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        VoltageSensor controlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);



        shooter Shooter = new shooter(hardwareMap);
        feeder_v1 Feeder = new feeder_v1(hardwareMap);
        jambes Chassis = new jambes(hardwareMap);
        bouche_intake Intake = new bouche_intake(hardwareMap);
        viseur Volet = new viseur(hardwareMap);

        Apriltag_reader aprilJoke = new Apriltag_reader();
        Vrai_vision objectProcessor = new Vrai_vision();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilJoke)
                .addProcessor(objectProcessor)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .build();


        IMU.Parameters imu_parameters;
        imu_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(0, 0, 90)));
        imu.initialize(imu_parameters);
        imu.resetYaw();



        double robotOrienDegrees;
        float forward;
        float turn;
        double seuil_shootter = 12.3;
        double voltage = controlHub_VoltageSensor.getVoltage();
        //PowerShooter = (PowerShooter * seuil_shootter) / voltage;
        //Power_bankMid = (Power_bankMid * seuil_shootter) / voltage;
        //powerMid = (Power_bankMid * seuil_shootter) / voltage;
        //Power_far = (Power_far * seuil_shootter) / voltage;
        boolean isShooting = false;
        int nbeBallesIn = 0;
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

            robotOrienDegrees = imu.getRobotYawPitchRollAngles().getYaw();


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
            double valueLeftMotor = forward + turn ;
            double valueRightMotor = forward - turn ;
            if(shooter.isShooting){
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
            Shooter.regleurVeloShooteur(
                    gamepad1, gamepad2);

            Shooter.setPIDFcoefficient(gamepad1, gamepad2);


            //vising
            Volet.viseur(gamepad1, gamepad2);


            //feeder
            Feeder.Feeder(gamepad1);


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


            telemetry.addData("Velocité programmé Shooter =", Shooter.getVeloShooter());
            telemetry.addData("PIDF shooter =", Shooter.getpidfcoeff());
            telemetry.addData("Vrai vélocité Shooter =", Shooter.shooter.getVelocity());
            telemetry.addData("vrai puissance shooter", Shooter.getpower());
            telemetry.addData("Position Viseur ", viseur.viseur.getPosition());
            telemetry.addData("posfeed", Feeder.getposition());
            telemetry.addData("Distance", compteurBalle.getDistance(DistanceUnit.CM));
            telemetry.addLine(shooter.isShooting ? "Shooter allumé" : "Shooter éteint");



            if (actual_april != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", actual_april.id, actual_april.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", actual_april.ftcPose.x, actual_april.ftcPose.y, actual_april.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", actual_april.ftcPose.pitch, actual_april.ftcPose.roll, actual_april.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", actual_april.ftcPose.range, actual_april.ftcPose.bearing, actual_april.ftcPose.elevation));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        actual_april.robotPose.getPosition().x,
                        actual_april.robotPose.getPosition().y,
                        actual_april.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        actual_april.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        actual_april.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        actual_april.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else telemetry.addLine("aucun apriltag détécté");
            telemetry.update();
          }
        visionPortal.close();
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





