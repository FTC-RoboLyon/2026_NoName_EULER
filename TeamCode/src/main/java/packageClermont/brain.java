package packageClermont;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static packageClermont.organe.Constant.FEEDER;
import static packageClermont.organe.Constant.INTAKE;
import static packageClermont.organe.Constant.LEFT_MOTOR;
import static packageClermont.organe.Constant.RIGHT_MORTOR;
import static packageClermont.organe.Constant.SHOOTER;
import static packageClermont.organe.Constant.VISEUR;


import packageClermont.organe.Feeder;
import packageClermont.organe.Viseur;
import packageClermont.organe.bouche;
import packageClermont.organe.jambes;
import packageClermont.organe.Shooter;

@TeleOp(name = "Compet_brain", group = "Euler")
public class brain extends LinearOpMode {
    public double VeloFion;
    public PIDFCoefficients PidCoef;
    double value_jambeDroite;
    double value_jambeGauche;
    float veloProg;

    boolean isShooting = false;
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor jambe_droite = hardwareMap.get(DcMotor.class,RIGHT_MORTOR);
        DcMotor jambe_gauche = hardwareMap.get(DcMotor.class , LEFT_MOTOR);
        DcMotorEx machoire = hardwareMap.get(DcMotorEx.class, INTAKE);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class , SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);



        jambes jambes = new jambes(jambe_droite, jambe_gauche);
        bouche bouche = new bouche(machoire);
        Shooter shooter1 = new Shooter(shooter);
        Feeder feeder1 = new Feeder(feeder);
        Viseur viseur1 = new Viseur(viseur);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            float leftY = -gamepad1.left_stick_y;
            float rightX = -gamepad1.right_stick_x/2;
            value_jambeDroite = leftY + rightX;
            value_jambeGauche = leftY - rightX;
            if(gamepad1.rightBumperWasPressed()){
                isShooting = !isShooting;
            }
            if(isShooting){
                value_jambeDroite = value_jambeDroite/2;
                value_jambeGauche = value_jambeGauche/2;
            }
            jambes.jambage(value_jambeDroite, value_jambeGauche);
            bouche.manger(gamepad1.left_bumper, gamepad1.left_trigger);
            shooter1.Tir(gamepad1.b,
                    gamepad1.a,
                    gamepad1.y,
                    isShooting,
                    gamepad1.right_trigger,
                    gamepad2.dpadLeftWasPressed(),
                    gamepad2.dpadRightWasPressed(),
                    gamepad2.dpadUpWasPressed(),
                    gamepad2.dpadDownWasPressed(),
                    gamepad2.aWasPressed(),
                    gamepad2.bWasPressed(),
                    gamepad2.yWasPressed(),
                    gamepad2.xWasPressed());
            veloProg = shooter1.veloShooter(gamepad1.b,
                    gamepad1.a,
                    gamepad1.y,
                    gamepad2.dpadLeftWasPressed(),
                    gamepad2.dpadRightWasPressed(),
                    gamepad2.dpadUpWasPressed(),
                    gamepad2.dpadDownWasPressed());

            feeder1.grosseCommition(gamepad1.xWasPressed(), gamepad1.xWasReleased());
            viseur1.visage(gamepad1.a, gamepad1.b, gamepad1.y, gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed());
            VeloFion = shooter.getVelocity();
            PidCoef = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("P",PidCoef.p);
            telemetry.addData("velo programmée", veloProg);
            telemetry.addData("velo", VeloFion);
            telemetry.addData("posviseur", viseur1.viseur.getPosition());
            telemetry.update();



        }
    }
}