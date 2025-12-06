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
    final Servo viseur;


    public Driver(DcMotor leftMotor1, DcMotor rightMotor1, DcMotor intake1, DcMotor shooter1, Servo feeder1, Servo viseur1) {
        left_motor = leftMotor1;
        right_motor = rightMotor1;
        intake = intake1;
        shooter = shooter1;
        feeder = feeder1;
        viseur = viseur1;

        this.left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(float valueLeftMotor, float valueRightMotor, boolean a) {
        if (a) {
            valueLeftMotor /= 2;
            valueRightMotor /= 2;
        }
        left_motor.setPower(valueLeftMotor);
        right_motor.setPower(valueRightMotor);
    }
    public float limitateur (float valueLeftMotor, float valueRightMotor, boolean a){
        if (a) {
            valueLeftMotor /= 2;
            valueRightMotor /= 2;
        }
        return valueLeftMotor;
    }

    public void intake(int puissanceIntake, boolean left_bumper, boolean b) {
        if (left_bumper) {
            intake.setPower(puissanceIntake);
        } else if (b) {
            intake.setPower(-puissanceIntake);
        } else {
            intake.setPower(0);
        }
    }

    public void shooter(int velocityShooter, boolean right_bumper, boolean y) {
        if (right_bumper) {
            ((DcMotorEx) shooter).setVelocity(velocityShooter);
        } else if (y) {
            ((DcMotorEx) shooter).setVelocity(-velocityShooter);
        } else {
            ((DcMotorEx) shooter).setVelocity(0);
        }
    }


    public void inverseurIntake(int puissanceIntake, boolean b) {
        if (b) {
            puissanceIntake = -puissanceIntake;
        }

    }

    public void InverseurShooter(int velocityShooter, boolean y) {
        if (y) {
            velocityShooter = -velocityShooter;
        }
    }

    public int regleurPuissanceShooter(int velocityShooter, boolean fleche_haut, boolean fleche_bas) {
        if (fleche_haut) {
            velocityShooter += 100;
        } else if (fleche_bas) {
            velocityShooter -= 100;
        }
        return velocityShooter;
    }

    // je sais pas encore si on a besoin de mettre == true dc si la fleche bas marche pas c'est pour ca
    public void positionsShooter(int velocityShooter, int velocityShooterPos1, int velocityShooterPos2, boolean Fgauche, boolean Fdroite) {
        if (Fgauche) {
            velocityShooter = velocityShooterPos1;
        } else if (Fdroite) {
            velocityShooter = velocityShooterPos2;
        }
    }

    public void feeder(boolean xpr, boolean xrl) {
        if (xpr) {
            feeder.setPosition(0.2);

        } else if (xrl) {
            feeder.setPosition(0);
        }
    }

    public double viseur(boolean a2, boolean b, boolean FG2, boolean FD2, double posviseur) {
        if (a2) {
            posviseur = 0;
        } else if (FG2) {
            posviseur += 0.05;
        } else if (FD2){
            posviseur -= 0.05;
        } else if (b){
            posviseur = 1;
        }
        return posviseur;
    }
}
