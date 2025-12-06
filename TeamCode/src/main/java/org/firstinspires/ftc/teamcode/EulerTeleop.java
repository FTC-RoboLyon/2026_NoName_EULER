package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.FEEDER;
import static org.firstinspires.ftc.teamcode.euler.Constant.INTAKE;
import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.SHOOTER;
import static org.firstinspires.ftc.teamcode.euler.Constant.VISEUR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.euler.Driver;
import org.opencv.core.Core;

import java.util.Objects;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Driver myRobotDriver = new Driver(left_motor, right_motor, intake, shooter, feeder, viseur);

        int puissanceIntake = 1;
        int velocityShooter = 5100;
        int velocityShooterPos1 = 0;
        int velocityShooterPos2 = 0;
        int variableInverseurIntake = 0;
        int variableInverseurShooter = 0;
        boolean right_bumper = gamepad1.right_bumper;
        boolean left_bumper = gamepad1.left_bumper;
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean a2 = gamepad2.a;
        boolean b2 = gamepad2.b;
        boolean xpr = gamepad1.xWasPressed();
        boolean xrl = gamepad1.xWasReleased();
        boolean fleche_haut = gamepad1.dpad_up;
        boolean fleche_bas = gamepad1.dpad_down;
        boolean Fgauche = gamepad1.dpad_left;
        boolean Fdroite = gamepad1.dpad_right;

        feeder.setPosition(0);

        while (opModeIsActive()) {
            float turn = -gamepad1.right_stick_x;
            float forward = -gamepad1.left_stick_y;
            float valueLeftMotor = forward - turn;
            float valueRightMotor = forward + turn;

            telemetry.addData("Gamepad", "left:" + turn);
            telemetry.addData("Gamepad", "right:" + forward);
            telemetry.addData("Puissance Shooter =", "velocityShooter");
            telemetry.update();

            myRobotDriver.limitateur(valueLeftMotor, valueRightMotor, gamepad1.a);
            myRobotDriver.drive(valueLeftMotor, valueRightMotor);
            myRobotDriver.intake(puissanceIntake, gamepad1.left_bumper, gamepad1.b);
            //myRobotDriver.positionsShooter(velocityShooter, velocityShooterPos1, velocityShooterPos2, Fgauche, Fdroite);
            //myRobotDriver.règleurPuissanceShooter(velocitysooter, fleche_haut, fleche_bas);
            myRobotDriver.shooter(velocityShooter, gamepad1.right_bumper, gamepad1.y);
            //myRobotDriver.feeder(xpr, xrl);
        }
    }
}


//Ce qui est en commentaire est à mettre apres avoir déjà regardé si le shooter et l'intake marchent
//Parce que sinon ça encombre le code et si y'a des erreurs, on saura pas d'où elles viennent