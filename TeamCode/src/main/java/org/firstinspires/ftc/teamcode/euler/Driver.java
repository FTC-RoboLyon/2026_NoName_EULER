package org.firstinspires.ftc.teamcode.euler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Driver {

    final DcMotor left_motor;
    final DcMotor right_motor;
    final DcMotor intake;
    final DcMotor shooter;
    

    public Driver(DcMotor leftMotor1, DcMotor rightMotor1, DcMotor intake1, DcMotor shooter1) {
        left_motor = leftMotor1;
        right_motor = rightMotor1;
        intake = intake1;
        shooter = shooter1;

        this.left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void InverseurShooter(int puissanceShooter, int variableInverseurShooter, boolean y){
        if(y){
            if(variableInverseurShooter == 0){
                puissanceShooter = -puissanceShooter;
                variableInverseurShooter = 1;
            }
        }else{
            variableInverseurShooter = 0;
        }
    }

    public void règleurPuissanceShooter(int puissanceShooter, boolean fleche_haut, boolean fleche_bas){
        if(fleche_haut == true){
            puissanceShooter += 0.1;
        } else if (fleche_bas)
            {puissanceShooter -= 0.1;
        }
    }
// je sais pas encore si on a besoin de mettre == true dc si la fleche bas marche pas c'est pour ca
    public void positionsShooter(int puissanceShooter, int puissanceShooterPos1, int puissanceShooterPos2, boolean Fgauche, boolean Fdroite){
        if(Fgauche){
            shooter.setPower(puissanceShooterPos1);
        } else if (Fdroite) {
            shooter.setPower(puissanceShooterPos2);
        }
    }
}
