package org.firstinspires.ftc.teamcode.euler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Driver {

    final DcMotor leftMotor;
    final DcMotor rightMotor;

    public Driver(DcMotor leftMotor1, DcMotor rightMotor1) {
        this.leftMotor = leftMotor1;
        this.rightMotor = rightMotor1;

        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive(float left, float right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }
}
