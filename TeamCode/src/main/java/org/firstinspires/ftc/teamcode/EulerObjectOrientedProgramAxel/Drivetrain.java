package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {
    public static final double KP_AUTO_ALIGN = 0.5; // TUNEME
    public static final double KD_AUTO_ALIGN = 0.5; // TUNEME

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private double leftMotorPower;
    private double rightMotorPower;
    private ElapsedTime PDTimer = new ElapsedTime();
    private static double lastErrorPD;
    private static double lastTimePD;
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

    public void DriveAlongATarget(double error, double forward){

        double pTerm = KP_AUTO_ALIGN * error;

        double actualTime = PDTimer.milliseconds();
        double dTerm = KD_AUTO_ALIGN * ((error - lastErrorPD)/actualTime - lastTimePD);

        double turn = pTerm + dTerm;

        leftMotorPower = forward + turn;
        rightMotorPower = forward - turn;

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        lastErrorPD = error;
        lastTimePD = actualTime;

    }



}
