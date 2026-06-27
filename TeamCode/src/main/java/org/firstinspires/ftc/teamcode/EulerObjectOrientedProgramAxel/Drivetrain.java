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
    private double previousPDError = 0.0;
    private double previousPDTime = 0.0;
    public Drivetrain(HardwareMap hmap){
        leftMotor = hmap.get(DcMotor.class, "left motor");
        rightMotor = hmap.get(DcMotor.class, "right motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PDTimer.startTime();
    }
    public void Drive (double turn, double forward){
        leftMotorPower = forward - turn;
        rightMotorPower = forward + turn;

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

    }

    public void AlignWithTarget(double error, double forward){

        double pTerm = KP_AUTO_ALIGN * error;

        double actualTime = PDTimer.milliseconds();
        double dTerm = KD_AUTO_ALIGN * ((error - previousPDError)/(actualTime - previousPDTime));//et il se passe quoi s'il n'y avait pas de previousPDTime ou de previousPDError

        double turn = pTerm + dTerm;

        leftMotorPower = forward - turn;
        rightMotorPower = forward + turn;

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        previousPDError = error;
        previousPDTime = actualTime;

    }



}
