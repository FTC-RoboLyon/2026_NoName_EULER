package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

    public void intake(int puissanceIntake, boolean left_bumper, double Left_Trig) {
        if (left_bumper) {
            intake.setPower(puissanceIntake);
        } else if (Left_Trig > 0.3) {
            intake.setPower(-puissanceIntake);
        } else {
            intake.setPower(0);
        }
    }

    public void shooter(int velocityShooter, boolean right_bumper, double right_Trig, double shoot_velo) {
        if (right_bumper) {
            ((DcMotorEx) shooter).setVelocity(velocityShooter);
        } else if (right_Trig > 0.3 || shoot_velo > 0){
            ((DcMotorEx) shooter).setVelocity(-50);
        } else {
            ((DcMotorEx) shooter).setVelocity(0);
        }
    }

    public int regleurPuissanceShooter(int velocityShooter, int PosTirNear, int PosTirFar, boolean fleche_haut, boolean fleche_bas, boolean b, boolean y) {
        if (fleche_haut) {
            velocityShooter += 100;
        } else if (fleche_bas) {
            velocityShooter -= 100;
        } else if (b) {
            velocityShooter = PosTirNear;
        } else if (y) {
            velocityShooter = PosTirFar;
        }
        return velocityShooter;
    }

    // je sais pas encore si on a besoin de mettre == true dc si la fleche bas marche pas c'est pour ca


    public void feeder(boolean xpr, boolean xrl) {
        if (xpr) {
            feeder.setPosition(0.2);

        } else if (xrl) {
            feeder.setPosition(0);
        }
    }

    public double viseur(boolean a, boolean b, boolean y, boolean FG2, boolean FD2, double posviseur, double posTir1, double posTir2) {
        if (a) {
            posviseur = 0.3;
        } else if (b){
            posviseur = posTir2;
        } else if (y) {
            posviseur = posTir1;
        } else if (FD2) {
            posviseur += 0.005;
        } else if (FG2){
            posviseur -= 0.005;
        }
        return posviseur;
    }

    public float drivePourDefit(float turn){
        turn /= 2;
        return turn;
    }
}
