package org.firstinspires.ftc.teamcode.CommandBase.Subsystem.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics.Pose2d;

public class Drivetrain_hardware {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    private double previousLeftPodValue = 0;
    private double previousRightPodValue = 0;
    private double previousStrafePodValue = 0;

    private double robotX = 0; //in meters
    private double robotY = 0; //in meters
    private double robotHeading = 0; //in radiants

    public Drivetrain_hardware(HardwareMap hmap){
        frontLeftMotor = hmap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hmap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hmap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hmap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void applyMotorsPower(double xPower, double yPower, double rotationPower){

        double maxMotorValue = Math.max(Math.abs(rotationPower) + Math.abs(xPower) + Math.abs(yPower), 1);

        double frontLeftPower = (xPower - rotationPower - yPower) / maxMotorValue;
        double frontRightPower = (xPower + rotationPower + yPower) / maxMotorValue;
        double backLeftPower = (xPower - rotationPower + yPower) / maxMotorValue;
        double backRightPower = (xPower + rotationPower - yPower) / maxMotorValue;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    public void update(){

        double leftPodValue = frontLeftMotor.getCurrentPosition() * Drivetrain_Constant.METERS_PER_TICK;
        double rightPodValue = frontRightMotor.getCurrentPosition() * Drivetrain_Constant.METERS_PER_TICK;
        double strafePodValue = backRightMotor.getCurrentPosition() * Drivetrain_Constant.METERS_PER_TICK;

        double dLeftValue = leftPodValue - previousLeftPodValue;
        double dRightValue = rightPodValue - previousRightPodValue;
        double dStrafeValue = strafePodValue - previousStrafePodValue;

        double dHeading = (dRightValue - dLeftValue)/ Drivetrain_Constant.E;

        double forward = (dLeftValue + dRightValue)/2;
        double strafe = dStrafeValue - dHeading * Drivetrain_Constant.ES;

        double deltaX = Math.cos(robotHeading)*forward - Math.sin(robotHeading)*strafe;
        double deltaY = Math.sin(robotHeading)*forward + Math.cos(robotHeading)*strafe;

        robotX += deltaX;
        robotY += deltaY;
        robotHeading += dHeading;

        //if (robotHeading > 2 * Math.PI){
        //    robotHeading -= 2 * Math.PI;
        //}
        //else if (robotHeading < 2 * Math.PI){
        //
        //}
        // bon je voulais faire en sorte que l'orientation dépasse pas 2pi mais jarrive pas je met en TODO

        previousLeftPodValue = leftPodValue;
        previousRightPodValue = rightPodValue;
        previousStrafePodValue = strafePodValue;
    }

    public void setPose(Pose2d startPos){
        robotX = startPos.xMeters;
        robotY = startPos.yMeters;
        robotHeading = startPos.headingRadians;
    }

    public double getDistanceToAPoint(Pose2d point){
        return Math.sqrt( Math.pow(point.xMeters - robotX, 2) + Math.pow(point.yMeters - robotY, 2));
    }

    public double getRobotHeading(){
        return robotHeading;
    }
    public double getRobotX(){
        return robotX;
    }
    public double getRobotY(){
        return robotY;
    }
}
