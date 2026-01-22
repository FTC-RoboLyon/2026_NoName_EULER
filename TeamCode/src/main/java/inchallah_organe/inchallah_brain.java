package inchallah_organe;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import static inchallah_organe.inchallah.Constant.FEEDER;
import static inchallah_organe.inchallah.Constant.INTAKE;
import static inchallah_organe.inchallah.Constant.LEFT_MOTOR;
import static inchallah_organe.inchallah.Constant.RIGHT_MORTOR;
import static inchallah_organe.inchallah.Constant.SHOOTER;
import static inchallah_organe.inchallah.Constant.VISEUR;
import static inchallah_organe.inchallah.trouDuFion.p;
import static inchallah_organe.inchallah.trouDuFion.i;
import static inchallah_organe.inchallah.trouDuFion.d;
import static inchallah_organe.inchallah.trouDuFion.f;


import static inchallah_organe.inchallah.Constant.VISEUR;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import inchallah_organe.inchallah.Intestin;
import inchallah_organe.inchallah.Viseur;
import inchallah_organe.inchallah.bouche;
import inchallah_organe.inchallah.jambes;
import inchallah_organe.inchallah.trouDuFion;

@TeleOp(name = "Inchallah_brain", group = "Euler")
public class inchallah_brain extends LinearOpMode {
    public double VeloFion;
    public PIDFCoefficients PidCoef;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor jambe_droite = hardwareMap.get(DcMotor.class,RIGHT_MORTOR);
        DcMotor jambe_gauche = hardwareMap.get(DcMotor.class , LEFT_MOTOR);
        DcMotorEx machoire = hardwareMap.get(DcMotorEx.class, INTAKE);
        DcMotorEx fion = hardwareMap.get(DcMotorEx.class , SHOOTER);
        Servo rectum = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);



        jambes jambes = new jambes(jambe_droite, jambe_gauche);
        bouche bouche = new bouche(machoire);
        trouDuFion fesse = new trouDuFion(fion);
        Intestin intestin = new Intestin(rectum);
        Viseur viseur1 = new Viseur(viseur);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            jambes.jambage(gamepad1.right_stick_x, gamepad1.left_stick_y);
            bouche.manger(gamepad1.left_bumper, gamepad1.left_trigger);
            fesse.caca(gamepad1.b,
                    gamepad1.a,
                    gamepad1.y,
                    gamepad1.rightBumperWasPressed(),
                    gamepad1.right_trigger,
                    gamepad2.dpadLeftWasPressed(),
                    gamepad2.dpadRightWasPressed(),
                    gamepad2.dpadUpWasPressed(),
                    gamepad2.dpadDownWasPressed(),
                    gamepad2.aWasPressed(),
                    gamepad2.bWasPressed(),
                    gamepad2.yWasPressed(),
                    gamepad2.xWasPressed());
            intestin.grosseCommition(gamepad1.xWasPressed(), gamepad1.xWasReleased());
            viseur1.visage(gamepad1.a, gamepad1.b, gamepad1.y, gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed());

            VeloFion = fion.getVelocity();
            PidCoef = fion.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("P",PidCoef.p);
            telemetry.addData("velo", VeloFion);

        }
    }
}