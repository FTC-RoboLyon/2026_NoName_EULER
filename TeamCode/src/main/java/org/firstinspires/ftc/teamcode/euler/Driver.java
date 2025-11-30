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

    public void shooter() {
        if(gamepad1.right_bumper) {
            shooter.setPower(1);
        } else if (gamepad.right_trigger) {
            shooter.setPower(-1);
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
                variableInverseurIntake == 1;
            }
        }else{
            variableInverseurIntake == 0;
        }
    }
}
