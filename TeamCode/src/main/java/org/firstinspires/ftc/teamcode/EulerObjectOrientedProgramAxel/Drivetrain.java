package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.utils;

public class Drivetrain {
    public static final double KP_AUTO_ALIGN = 0.5; // TUNEME mtn que j'y pense quelle est la différence entre lui et KP_TURN (d'ailleurs j'aurais plutot mit heading)
    public static final double KD_AUTO_ALIGN = 0.5; // TUNEME comme au dessus

    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    public final int TICKS_PER_REVOLUTION = 8192 ;
    // Tune this to the number of tick your captor register per wheel revolution
    //Alors je suis presque sûr qu'on dit plutôt sensor et ce même sensor est même un encoder
    public final double WHEEL_RADIUS = 0.45; //TUNEME in meters, c'est quoi comme roue ca au juste ??? (c peut etre bon mais ca m'a l'air bien grand)
    public final double METERS_PER_TICK = (WHEEL_RADIUS * Math.PI * 2) / TICKS_PER_REVOLUTION;
    public final double E = 5.0; // in meters, si ton entraxe fait 5m j'ose même pas imaginer la taille de ton robot
    public final double ES = 5.0; //in meters, c BIIIIIIIIIIG ;)
    public final static double KP_STRAFE = 0.25; //TUNEME
    public final static double KP_FORWARD = 0.25; //TUNEME
    public final static double KP_TURN = 0.25; //TUNEME
    public final static double KD_STRAFE = 0.25; //TUNEME
    public final static double KD_FORWARD = 0.25; //TUNEME
    public final static double KD_TURN = 0.25; //TUNEME
    public final static double TOLERANCE_X_AND_Y = 0.05; //TUNEME IN METERS
    public final static double TOLERANCE_HEADING = 0.25; //TUNEME, but what is the unit ??? 0.25 rad = 14 deg, fait bien par rapport au SI


    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private ElapsedTime PIDTimer = new ElapsedTime();
    private double previousPDError = 0.0;
    private double previousPDTime = 0.0;
    private double previousGoPosTime = 0.0;
    double previousLeftValue = 0; //Left quoi ?
    double previousRightValue = 0; //what Right ?
    double previousStrafeValue = 0; //Que Strafe ?
    double robotX = 0;
    double robotY = 0;
    double robotHeading = 0;
    // tune all the 3 values above to your robot starting pose (also well thought, you can just add a TUNEME mention)
    double previousXError = 0;
    double previousYError = 0;
    double previousHeadingError = 0;

    public Drivetrain(HardwareMap hmap){

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

        PIDTimer.startTime();
    }
    public Drivetrain (HardwareMap hmap, SparkFunOTOS.Pose2D startPos){
        this(hmap);
        robotX = startPos.x;
        robotY = startPos.y;
        robotHeading = startPos.h;
    }
    public void Drive (double turn, double forward, double strafe, boolean fieldOriented){

        if (fieldOriented){
            double forwardCopy = forward;
            forward = Math.cos(robotHeading)*forwardCopy + Math.sin(robotHeading)*strafe;
            strafe = -Math.sin(robotHeading)*forwardCopy + Math.cos(robotHeading)*strafe;
        }

        double maxMotorValue = Math.max(Math.abs(turn) + Math.abs(forward) + Math.abs(strafe), 1);

        frontLeftPower = (forward - turn - strafe) / maxMotorValue;
        frontRightPower = (forward + turn + strafe) / maxMotorValue;
        backLeftPower = (forward - turn + strafe) / maxMotorValue;
        backRightPower = (forward + turn - strafe) / maxMotorValue;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }


    /**
     * A function that allows the robot to move to a given point of coordinates (xTarget, yTarget) and head to a given heading target.
     * Return if the robot has arrived yet using tolerances.
     * @param xTarget X coordinate of the target point (in meters)
     * @param yTarget Y coordinate of the target point (in meters)
     * @param headingTarget heading target of the robot (in radians)
     * @return if the robot has arrived yet using tolerances (true : yes; false : no)
     */
    public boolean goToPos (double xTarget, double yTarget, double headingTarget) {
        //return true if the robot is already at the giving target point and heading
        if (utils.IsInRange(robotX, xTarget, TOLERANCE_X_AND_Y)
                && utils.IsInRange(robotY, yTarget, TOLERANCE_X_AND_Y)
                && utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING))
        {
            return true;
        }

        double xError = xTarget - robotX;
        double yError = yTarget - robotY;
        double headingError = headingTarget - robotHeading;

        double xErrorCopy = xError;
        xError = Math.cos(robotHeading)*xErrorCopy + Math.sin(robotHeading)*yError; //Bah non dcp ca ca represente plus la x error mais plutot la fwd error
        yError = -Math.sin(robotHeading)*xErrorCopy + Math.cos(robotHeading)*yError;//same

        double pTermX = KP_FORWARD * xError; //et dcp c'est plus le TermX non plus
        double pTermY = KP_STRAFE * yError;//same
        double pTermHeading = KP_TURN * headingError;

        double actualTime = PIDTimer.milliseconds();
        double dTermX = KD_FORWARD * ((xError - previousXError)/(actualTime - previousGoPosTime));
        double dTermY = KD_STRAFE * ((yError - previousYError)/(actualTime - previousGoPosTime));
        double dTermHeading = KD_TURN * ((headingError - previousHeadingError)/(actualTime - previousGoPosTime));

        double forward = pTermX + dTermX;
        double strafe = pTermY + dTermY;
        double turn = pTermHeading + dTermHeading;

        Drive(turn, forward, strafe, false);

        previousXError = xError;
        previousYError = yError;
        previousHeadingError = headingError;
        previousGoPosTime = actualTime;

        return false;
    }

    public void headToTarget(double error, double forward, double strafe){

        double pTerm = KP_AUTO_ALIGN * error;

        double actualTime = PIDTimer.milliseconds();
        double dTerm = KD_AUTO_ALIGN * ((error - previousPDError)/(actualTime - previousPDTime));//et il se passe quoi s'il n'y avait pas de previousPDTime ou de previousPDError
        //Tu m'as ignoré là :(

        double turn = pTerm + dTerm;

        Drive(turn, forward, strafe, true);

        previousPDError = error;
        previousPDTime = actualTime;
    }

    public void actualiseRobotPos (){

        double leftPodValue = frontLeftMotor.getCurrentPosition() * METERS_PER_TICK;
        double rightPodValue = frontRightMotor.getCurrentPosition() * METERS_PER_TICK;
        double strafePodValue = backRightMotor.getCurrentPosition() * METERS_PER_TICK;

        double dLeftValue = leftPodValue - previousLeftValue;
        double dRightValue = rightPodValue - previousRightValue;
        double dStrafeValue = strafePodValue - previousStrafeValue;

        double dHeading = (dRightValue - dLeftValue)/ E;
        robotHeading += dHeading;

        double forward = (dLeftValue + dRightValue)/2;
        double strafe = dStrafeValue - dHeading * ES;

        double deltaX = Math.cos(robotHeading)*forward - Math.sin(robotHeading)*strafe;
        double deltaY = Math.sin(robotHeading)*forward + Math.cos(robotHeading)*strafe;

        robotX += deltaX;
        robotY += deltaY;

        previousLeftValue = leftPodValue;
        previousRightValue = rightPodValue;
        previousStrafeValue = strafePodValue;
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
