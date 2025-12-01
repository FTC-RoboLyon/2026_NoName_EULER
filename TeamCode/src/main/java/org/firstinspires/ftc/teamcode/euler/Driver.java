package org.firstinspires.ftc.teamcode.euler;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    }

    public void drive(float valueLeftMotor, float valueRightMotor) {
        left_motor.setPower(valueLeftMotor);
        right_motor.setPower(valueRightMotor);
    }

    public void intake(int puissanceIntake) {
        if(gamepad1.left_bumper){
            intake.setPower(puissanceIntake);
        }else {
            intake.setPower(0);
        }
    }

    public void shooter(int puissanceShooter) {
        if(gamepad1.right_bumper) {
            shooter.setPower(puissanceShooter);
        } else {
            shooter.setPower(0);
        }
    }

    public void limitateur(float valueLeftMotor, float valueRightMotor) {
        if(gamepad.a){
            valueLeftMotor = valueLeftMotor/2;
            valueRightMotor = valueRightMotor/2;
        }
    }
    public void inverseurIntake(int puissanceIntake, int variableInverseurIntake){
        if(gamepad1.b){
            if(variableInverseurIntake == 0){
                puissanceIntake = -puissanceIntake;
                variableInverseurIntake = 1;
            }
        }else{
            variableInverseurIntake = 0;
        }
    }
    public void InverseurShooter(int puissanceShooter, int variableInverseurShooter){
        if(gamepad1.y){
            if(variableInverseurShooter == 0){
                puissanceShooter = -puissanceShooter;
                variableInverseurShooter = 1;
            }
        }else{
            variableInverseurShooter = 0;
        }
    }

    public void règleurPuissanceShooter(int puissanceShooter){
        if(gamepad1.right_trigger){
            puissanceShooter = puissanceShooter+0.1;
        } else if (gamepad1.left_trigger) {
            puissanceShooter = puissanceShooter-0.1;
        }
    }

    public void positionsShooter(int puissanceShooter, int puissanceShooterPos1, int puissanceShooterPos2){
        if(gamepad2.a){
            shooter.setPower(puissanceShooterPos1);
        } else if (gamepad2.b) {
            shooter.setPower(puissanceShooterPos2);
        }
    }
}
