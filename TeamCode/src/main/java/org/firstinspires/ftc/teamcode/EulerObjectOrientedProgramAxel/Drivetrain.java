package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private final DcMotor leftMotor;
    private DcMotor rightMotor;
    private double leftMotorPower;
    private double rightMotorPower;
    public enum Drive_mode {
        DRIVE,
        AUTO
    }
    public Drive_mode driveMode = Drive_mode.DRIVE;
    public Drivetrain(HardwareMap hmap){
        leftMotor = hmap.get(DcMotor.class, "left motor");
        rightMotor = hmap.get(DcMotor.class, "right motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void Drive (double turn, double forward){
        leftMotorPower = forward + turn;
        rightMotorPower = forward - turn;

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);
    }
}
