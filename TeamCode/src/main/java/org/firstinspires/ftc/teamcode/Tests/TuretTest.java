package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Camera.Camera;

@Config
@TeleOp
public class TuretTest extends OpMode {
    private DcMotor turretMotor;
    Camera camera;
    ElapsedTime timer;

    double previousTime = 0.0;
    double previousError = 0.0;

    double turretPower;
    public static double turretKP = 0.00012;
    public static double turretKD = 0.0023;
    public static double turretKI = 0.0000000029;
    double integrative = 0.0;
    double turretTolerance;

    double targetPos; //in ticks

    boolean firstIteration = true;



    public final static double TICKS_PER_DEGREES = (double) 32768 /405;

    @Override
    public void init(){

        //camera = new Camera(hardwareMap);

        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        timer = new ElapsedTime();
        timer.startTime();
    }

    @Override
    public void loop(){
        setTargetFromDegrees(90);
        goToTarget();

        telemetry.addData("Target", targetPos);
        telemetry.addData("Current", turretMotor.getCurrentPosition());
    }

    private void goToTarget(){
        double actualError = targetPos-turretMotor.getCurrentPosition();
        double pTerm = actualError * turretKP;
        double actualTime = timer.milliseconds();

        double dTime = actualTime - previousTime;

        if (firstIteration) {
            previousError = actualError;
            firstIteration = false;
        }

        double dTerm = turretKD * (actualError-previousError) / dTime;

        integrative += actualError * dTime;
        double iTerm = turretKI * integrative;

        turretPower = pTerm + dTerm + iTerm;
        turretMotor.setPower(clamp(-turretPower, -1, 1));

        previousError = actualError;
        previousTime = actualTime;
    }
    private void setTargetFromDegrees(double degrees){
        targetPos = degrees * TICKS_PER_DEGREES;
    }

    public double clamp(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
    }


}
