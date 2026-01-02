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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import Webcam_aldnc_yeux.vision;

@TeleOp(name = "ALDNC_brain", group = "Euler")
public class ALDNC_brain extends LinearOpMode {

    private IMU imu;
    private VoltageSensor ControlHub_VoltageSensor;

    vision vision;

    enum ModeRobot {Manuel, ChercheBalle, place_shoot}
    enum Pos_Balle {gauche, centre, droite, non_detected}

    static ModeRobot robot = ModeRobot.Manuel;
    static Pos_Balle balle = Pos_Balle.non_detected;

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
        DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);
        vision = new vision(hardwareMap);


        shooter Shooter = new shooter(shooter);
        feeder_v1 Feeder = new feeder_v1(feeder);
        jambes Chassis = new jambes(left_motor, right_motor);
        bouche_intake Intake = new bouche_intake(intake);
        viseur Volet = new viseur(viseur);

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
        double distance = compteurBalle.getDistance(DistanceUnit.CM);
        boolean isIntaking = false;
        boolean isShooting = false;
        int nbeBallesIn = 3;
        int vIntake = 0;
        boolean isFeeding = false;
        int a = 0;
        int b = 0;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            //camera
            if (vision.isObjectDetected()) {
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
            isIntaking = distance < 25;
            nbeBallesIn = Intake.nbeBalles(distance, nbeBallesIn);

            switch_states(gamepad2.x, gamepad2.y, gamepad2.b);
            switch (robot) {
                case Manuel:


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




                    if (gamepad1.bWasPressed()) {
                        PowerShooter = Power_bank;
                    } else if (gamepad1.yWasPressed()) {
                        PowerShooter = Power_far;
                    }


                    if (gamepad1.rightBumperWasPressed()) {
                        isShooting = !isShooting;
                    }
                    if (nbeBallesIn == 0) {
                        isShooting = false;
                    }

                    //shooting
                    Shooter.shooter(PowerShooter,
                            gamepad1.right_trigger,
                            isShooting);


                    //vising
                    Volet.viseur(gamepad1.a,
                            gamepad1.b,
                            gamepad1.y,
                            gamepad1.dpadRightWasPressed(),
                            gamepad1.dpadLeftWasPressed(),
                            posviseur,
                            posviseur_far,
                            posviseur_bank);


                    //feeder

                    isFeeding = Feeder.feederPara(gamepad1.xWasPressed(), nbeBallesIn, isFeeding, isShooting);
                    Feeder.feeder(isFeeding, isShooting);
                    Feeder.compteurBalles(a, nbeBallesIn, isFeeding, isShooting);


                case ChercheBalle:
                    switch (balle) {
                        case gauche:
                            Chassis.turn_antihoraire();
                        case droite:
                            Chassis.turn_horaire();
                        case centre:
                            Chassis.forward();
                            Intake.intake_simple();
                        case non_detected:
                            robot = ModeRobot.Manuel;
                    }
            }
            real_velo = ((DcMotorEx) shooter).getVelocity();
            distance = compteurBalle.getDistance(DistanceUnit.CM);
            telemetry.addData("Velocité programmé Shooter =", PowerShooter);
            telemetry.addData("Vrai vélocité Shooter =", real_velo);
            telemetry.addData("Position Viseur ", viseur.getPosition());
            telemetry.addData("Distance", compteurBalle.getDistance(DistanceUnit.CM));
            telemetry.addData("is intaking", isIntaking);
            telemetry.addData("Nbe Balles Inside Bot = ", nbeBallesIn);
            if (isShooting) {
                telemetry.addLine("Shooter allumé");
            } else if (!isShooting) {
                telemetry.addLine("Shooter éteint");
            }
            telemetry.update();
        }
        vision.stop();


    }

    public void switch_states(boolean gmp2_x, boolean gmp2_y, boolean gmp2_b) {
        if (gmp2_x) {
            robot = ModeRobot.Manuel;
        } else if (gmp2_y) {
            robot = ModeRobot.ChercheBalle;
        } else if (gmp2_b) {
            robot = ModeRobot.place_shoot;
        } else {
            robot = ModeRobot.Manuel;
        }
    }
}




