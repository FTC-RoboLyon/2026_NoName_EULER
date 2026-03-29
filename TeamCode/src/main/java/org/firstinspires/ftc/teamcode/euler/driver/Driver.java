package org.firstinspires.ftc.teamcode.euler.driver;

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


    /**
     * Drive the robot using arcade controls.
     *
     * @param left
     * @param right
     */
    public void drive(float left, float right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    public DriverState getState() {
        if (leftMotor.getPower() == 0 && rightMotor.getPower() == 0) {
            return DriverState.IDLE;
        } else {
            return DriverState.MOVING;
        }
    }
}
