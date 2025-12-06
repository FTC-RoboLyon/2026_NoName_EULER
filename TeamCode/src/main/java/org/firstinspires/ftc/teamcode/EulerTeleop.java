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
        double posviseur = 0;


        feeder.setPosition(0);

        while (opModeIsActive()) {
            float turn = -gamepad1.right_stick_x;
            float forward = -gamepad1.left_stick_y;
            float valueLeftMotor = forward - turn;
            float valueRightMotor = forward + turn;

            telemetry.addData("Puissance Shooter =", velocityShooter);
            telemetry.addData("Position Viseur ", viseur.getPosition());
            telemetry.update();


            myRobotDriver.drive(valueLeftMotor, valueRightMotor, gamepad2.x);
            myRobotDriver.intake(puissanceIntake, gamepad1.left_bumper, gamepad2.b);
            //myRobotDriver.positionsShooter(velocityShooter, velocityShooterPos1, velocityShooterPos2, Fgauche, Fdroite);
            velocityShooter = myRobotDriver.regleurPuissanceShooter(velocityShooter, gamepad1.dpadUpWasPressed(), gamepad1.dpadDownWasPressed());
            myRobotDriver.shooter(velocityShooter, gamepad1.right_bumper, gamepad1.y);
            myRobotDriver.feeder(gamepad1.xWasPressed(), gamepad1.xWasReleased());
            posviseur = myRobotDriver.viseur(gamepad1.a, gamepad1.b, gamepad1.dpadLeftWasPressed(), gamepad1.dpadRightWasPressed(), posviseur);
            viseur.setPosition(posviseur);
        }
    }
}


