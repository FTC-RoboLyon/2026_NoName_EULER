package inchallah_organe;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import static inchallah_organe.inchallah.Constant.RECTUM;
import static inchallah_organe.inchallah.Constant.MACHOIRE;
import static inchallah_organe.inchallah.Constant.JAMBE_GAUCHE;
import static inchallah_organe.inchallah.Constant.JAMBE_DOITE;
import static inchallah_organe.inchallah.Constant.FION;

import static inchallah_organe.inchallah.Constant.VISEUR;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import inchallah_organe.inchallah.Intestin;
import inchallah_organe.inchallah.bouche;
import inchallah_organe.inchallah.jambes;
import inchallah_organe.inchallah.trouDuFion;

@TeleOp(name = "Inchallah_brain", group = "Euler")
public class inchallah_brain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor jambe_droite = hardwareMap.get(DcMotor.class,JAMBE_DOITE);
        DcMotor jambe_gauche = hardwareMap.get(DcMotor.class , JAMBE_GAUCHE);
        DcMotorEx machoire = hardwareMap.get(DcMotorEx.class, MACHOIRE);
        DcMotor fion = hardwareMap.get(DcMotor.class , FION);
        Servo rectum = hardwareMap.get(Servo.class, RECTUM);



        jambes jambes = new jambes(jambe_droite, jambe_gauche);
        bouche bouche = new bouche(machoire);
        trouDuFion fesse = new trouDuFion(fion);
        Intestin Intestin = new Intestin(rectum);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            jambes.jambage(gamepad1.right_stick_x, gamepad1.left_stick_y);
            bouche.manger(gamepad1.left_bumper, gamepad1.left_trigger);
            fesse.caca(gamepad1.b, gamepad1.a, gamepad1.y, gamepad1.rightBumperWasPressed(), gamepad1.right_trigger);
            Intestin.grosseCommition(gamepad1.xWasPressed(), gamepad1.xWasReleased());

        }
    }
}