package packageClermont;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static packageClermont.organe.Constant.FEEDER;
import static packageClermont.organe.Constant.INTAKE;
import static packageClermont.organe.Constant.LEFT_MOTOR;
import static packageClermont.organe.Constant.RIGHT_MORTOR;
import static packageClermont.organe.Constant.SHOOTER;
import static packageClermont.organe.Constant.VISEUR;

import FRC_ALDNC.SubSystem.Apriltag_reader;
import packageClermont.organe.Feeder;
import packageClermont.organe.Viseur;
import packageClermont.organe.bouche;
import packageClermont.organe.jambes;
import packageClermont.organe.Shooter;
import packageClermont.organe.joySticks.joyStickY;
import packageClermont.organe.joySticks.joystickX;
@Config
@TeleOp(name = "Compet_brain", group = "Euler")
public class brain extends LinearOpMode {
    public double visionX;
    public double VeloFion;
    double rightX;
    double leftY;
    double powerShooter = 0;
    public PIDFCoefficients PidCoef;
    double value_jambeDroite;
    double value_jambeGauche;
    double veloProg;
    double x1 = 0;
    double y1 = 0;
    float x;
    float y;
    public  static  double vvision = 0.2;


    boolean isShooting = false;
    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor controlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        double seuil_shootter = 12.3;
        double voltage = controlHub_VoltageSensor.getVoltage();
        //PowerShooter = (PowerShooter * seuil_shootter) / voltage;
        //Power_bankMid = (Power_bankMid * seuil_shootter) / voltage;
        //powerMid = (Power_bankMid * seuil_shootter) / voltage;
        //Power_far = (Power_far * seuil_shootter) / voltage;
        boolean isShooting = false;
        int nbeBallesIn = 0;
        DcMotor jambe_droite = hardwareMap.get(DcMotor.class,RIGHT_MORTOR);
        DcMotor jambe_gauche = hardwareMap.get(DcMotor.class , LEFT_MOTOR);
        DcMotorEx machoire = hardwareMap.get(DcMotorEx.class, INTAKE);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class , SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        Apriltag_reader apriljoke = new Apriltag_reader(hardwareMap);



        jambes jambes = new jambes(jambe_droite, jambe_gauche);
        bouche bouche = new bouche(machoire);
        Shooter shooter1 = new Shooter(shooter);
        Feeder feeder1 = new Feeder(feeder);
        Viseur viseur1 = new Viseur(viseur);
        joyStickY joyStickY = new joyStickY(telemetry);
        joystickX joystickX = new joystickX(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            /*rightX = joystickX.joyStickXPara(gamepad1.right_stick_x, x1, gamepad1.left_stick_y, y1);
            leftY = joyStickY.joyStickYPara(gamepad1.right_stick_x, x1, gamepad1.left_stick_y, y1);
            x1 = rightX;
            y1 = leftY;

            if(gamepad1.left_bumper){
                leftY = leftY /1.5;
            }*/
            if(gamepad1.rightBumperWasPressed()){
                isShooting = !isShooting;
            }
            if(isShooting){
                rightX = rightX/1.3;
                leftY = leftY/1.3;
            }


            /*value_jambeDroite = rightX - leftY;
            value_jambeGauche = rightX + leftY;

            jambes.jambage(value_jambeDroite, value_jambeGauche, gamepad1);
            bouche.manger(gamepad1.left_bumper, gamepad1.left_trigger);*/

            apriljoke.updtade();
            visionX = apriljoke.getTagXNormalized(24);
            if(visionX > 0){
                jambe_droite.setPower(-vvision);
                jambe_gauche.setPower(vvision);
            }else if(visionX < 0){
                jambe_droite.setPower(vvision);
                jambe_gauche.setPower(-vvision);
            }else if(visionX == 0){
                jambe_droite.setPower(0);
                jambe_gauche.setPower(0);
            }
            shooter1.Tir_using_velo(gamepad1.b,
                    gamepad1.a,
                    gamepad1.y,
                    isShooting,
                    gamepad1.right_trigger,
                    gamepad1.dpadLeftWasPressed(),
                    gamepad1.dpadRightWasPressed(),
                    gamepad1.dpadUpWasPressed(),
                    gamepad1.dpadDownWasPressed());
            veloProg = shooter1.veloShooter(gamepad1.b,
                    gamepad1.a,
                    gamepad1.y,
                    gamepad1.dpadLeftWasPressed(),
                    gamepad1.dpadRightWasPressed(),
                    gamepad1.dpadUpWasPressed(),
                    gamepad1.dpadDownWasPressed());


            shooter1.p(gamepad2.dpadDownWasPressed(),
                    gamepad2.dpadRightWasPressed(),
                    gamepad2.dpadUpWasPressed(),
                    gamepad2.dpadLeftWasPressed(), telemetry);
            shooter1.d(gamepad2, telemetry);
            /*powerShooter = shooter1.powerShooter(gamepad2.aWasPressed(),
                    gamepad2.bWasPressed(),
                    gamepad2.yWasPressed(),
                    gamepad2.xWasPressed());*/
            /*if(gamepad1.dpadRightWasPressed()){
                powerShooter = powerShooter + 0.05;
            }else if(gamepad1.dpadLeftWasPressed()){
                powerShooter = powerShooter - 0.05;
            }else if(gamepad1.dpadUpWasPressed()){
                powerShooter = powerShooter + 0.1;
            }else if(gamepad1.dpadDownWasPressed()){
                powerShooter = powerShooter - 0.1;
            }
            if(isShooting){
                shooter.setPower(powerShooter);
            }else if(!isShooting){
                shooter.setPower(0);
            }*/

            feeder1.grosseCommition(gamepad1.xWasPressed(), gamepad1.xWasReleased());
            viseur1.visage(gamepad1.a, gamepad1.b, gamepad1.y, gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed());
            VeloFion = shooter.getVelocity();
            PidCoef = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("vitesse", shooter.getVelocity());
            telemetry.addData("velo programmée", veloProg);
            //telemetry.addData("p", shooter1.getP());
            telemetry.addData("bouton", gamepad2.dpad_down);
            //telemetry.addData("velo", VeloFion);
            //telemetry.addData("posviseur", viseur1.viseur.getPosition());
            //telemetry.addData("Tension", powerShooter);
            //telemetry.addData("value_jambe droite", value_jambeDroite);
            //telemetry.addData("value_jambe_gauche", value_jambeGauche);
            apriljoke.telemetry(telemetry);
            telemetry.update();



        }
    }
}