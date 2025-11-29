package org.firstinspires.ftc.teamcode.euler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Driver {

    final DcMotor left_motor;
    final DcMotor right_motor;

    public Driver(DcMotor leftMotor1, DcMotor rightMotor1) {
        left_motor = leftMotor1;
        right_motor = rightMotor1;

        this.left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(float tourner, float avancer) {
        left_motor.setPower(avancer);
        right_motor.setPower(avancer);
        if
    }
}
