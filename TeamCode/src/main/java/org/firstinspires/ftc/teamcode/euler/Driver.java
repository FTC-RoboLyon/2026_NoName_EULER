package org.firstinspires.ftc.teamcode.euler;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Core;

public class Driver {

    final DcMotor left_motor;
    final DcMotor right_motor;
    final DcMotor intake;
    final DcMotor shooter;
    final Servo feeder;
    

    public Driver(DcMotor leftMotor1, DcMotor rightMotor1, DcMotor intake1, DcMotor shooter1, Servo feeder1) {
        left_motor = leftMotor1;
        right_motor = rightMotor1;
        intake = intake1;
        shooter = shooter1;
        feeder = feeder1;

        this.left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(float valueLeftMotor, float valueRightMotor) {
        left_motor.setPower(valueLeftMotor);
        right_motor.setPower(valueRightMotor);
    }

    public void intake(int puissanceIntake, boolean left_bumper) {
        if(left_bumper){
            intake.setPower(puissanceIntake);
        }else {
            intake.setPower(0);
        }
    }

    public void shooter(int velocityShooter, boolean right_bumper) {
        if(right_bumper) {
            ((DcMotorEx) shooter).setVelocity (velocityShooter);
        } else {
            ((DcMotorEx) shooter).setVelocity (0);
        }
    }

    public void limitateur(float valueLeftMotor, float valueRightMotor, boolean a) {
        if(a){
            valueLeftMotor = valueLeftMotor/2;
            valueRightMotor = valueRightMotor/2;
        }
    }
    public void inverseurIntake(int puissanceIntake, int variableInverseurIntake, boolean b){
        if(b){
            if(variableInverseurIntake == 0){
                puissanceIntake = -puissanceIntake;
                variableInverseurIntake = 1;
            }
        }else{
            variableInverseurIntake = 0;
        }
    }
    public void InverseurShooter(int velocityShooter, int variableInverseurShooter, boolean y){
        if(y){
            if(variableInverseurShooter == 0){
                velocityShooter = -velocityShooter;
                variableInverseurShooter = 1;
            }
        }else{
            variableInverseurShooter = 0;
        }
    }

    public void règleurPuissanceShooter(int velocityShooter, boolean fleche_haut, boolean fleche_bas){
        if(fleche_haut){
            velocityShooter += 100;
        } else if (fleche_bas) {
            velocityShooter -= 100;
        }
    }
// je sais pas encore si on a besoin de mettre == true dc si la fleche bas marche pas c'est pour ca
    public void positionsShooter(int velocityShooter, int velocityShooterPos1, int velocityShooterPos2, boolean Fgauche, boolean Fdroite){
        if(Fgauche){
            velocityShooter = velocityShooterPos1;
        } else if (Fdroite) {
            velocityShooter = velocityShooterPos2;
        }
    }
    public void feeder( boolean xpr, boolean xrl){
        if (xpr){
            feeder.setPosition(1);

        } else if (xrl) {
            feeder.setPosition(0);
        }
    }
}
