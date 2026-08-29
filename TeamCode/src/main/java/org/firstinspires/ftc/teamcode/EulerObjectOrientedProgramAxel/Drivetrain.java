package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.utils;

public class Drivetrain {
    public static final double KP_AUTO_ALIGN = 0.5; // TUNEME
    public static final double KD_AUTO_ALIGN = 0.5; // TUNEME

    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    public final int TICKS_PER_REVOLUTION = 8192 ;
    // Tune this to the number of tick your captor register per wheel revolution
    public final double WHEEL_RADIUS = 0.45; //TUNEME in meters, ah ouais tu mets des sacres roues pour avoir 4.5m de rayon toi
    public final double METERS_PER_TICK = (WHEEL_RADIUS * Math.PI * 2) / TICKS_PER_REVOLUTION;
    public final double E = 5.0; // in meters
    public final double ES = 5.0; //in meters
    public final static double KP_STRAFE = 0.25; //TUNEME
    public final static double KP_FORWARD = 0.25; //TUNEME
    public final static double KP_TURN = 0.25; //TUNEME
    public final static double KD_STRAFE = 0.25; //TUNEME
    public final static double KD_FORWARD = 0.25; //TUNEME
    public final static double KD_TURN = 0.25; //TUNEME
    public final static double TOLERANCE_X_AND_Y = 0.05; //TUNEME IN METERS
    public final static double TOLERANCE_HEADING = 0.25; //TUNEME


    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double forward = 0.0, strafe = 0.0, turn = 0.0;
    private ElapsedTime goToPosTimer = new ElapsedTime();
    private ElapsedTime headingTimer = new ElapsedTime();
    private double previousGoPosTime = 0.0;
    private double previousHeadingTime = 0.0;
    double previousLeftValue = 0;
    double previousRightValue = 0;
    double previousStrafeValue = 0;
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

        goToPosTimer.startTime();
        headingTimer.startTime();
    }
    public Drivetrain (HardwareMap hmap, SparkFunOTOS.Pose2D startPos){
        this(hmap);
        robotX = startPos.x;
        robotY = startPos.y;
        robotHeading = startPos.h;
    }

    /**
     * Move the drivetrain using
     * @param Turn the power with which the robot will turn
     * @param Forward the power with which the robot will move forward
     * @param Strafe the power with which the robot will move sideway
     *               For the three parameters above, if we give the value of 1000.0, the precedents values given will be kept
     * @param fieldOriented if we want to transform Forward and Strafe power from field coordinate to robot coordinate
     */
    public void Drive (double Turn, double Forward, double Strafe, boolean fieldOriented){
        if(Forward != 1000.0 && Strafe != 1000.0){
            if (fieldOriented){
                forward = Math.cos(robotHeading)*Forward + Math.sin(robotHeading)*Strafe;
                strafe = -Math.sin(robotHeading)*Forward + Math.cos(robotHeading)*Strafe;
            }else{
                forward = Forward;
                strafe = Strafe;
            }
        }
        if (Turn != 1000.0)
            turn = Turn;

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


    //Les docs ca ressemble plutot a ca (meme si la elle peuvent encore etre mieux) :
    /**
     * A function that allows the robot to move to a given point of coordinates (xTarget, yTarget) while turning itself freely.
     * Return if the robot has arrived yet using tolerances.
     * @param xTarget X coordinate of the target point (in meters)
     * @param yTarget Y coordinate of the target point (in meters)
     * @param turn the rotation given directly to the robot (if you don't want the robot to turn just set it to 0)
     *                  For the parameter above, if we give the value of 1000.0, the precedent value given will be kept
     * @return if the robot has arrived yet using tolerances (true : yes; false : no)
     */
    public boolean goToPos (double xTarget, double yTarget, double turn) {
        //return true if the robot is already at the giving target point and heading
        if (utils.IsInRange(robotX, xTarget, TOLERANCE_X_AND_Y)
                && utils.IsInRange(robotY, yTarget, TOLERANCE_X_AND_Y)) {
            return true;
        }

        double xError = xTarget - robotX;
        double yError = yTarget - robotY;

        double xErrorCopy = xError;
        xError = Math.cos(robotHeading) * xErrorCopy + Math.sin(robotHeading) * yError;
        yError = -Math.sin(robotHeading) * xErrorCopy + Math.cos(robotHeading) * yError;

        double pTermX = KP_FORWARD * xError;
        double pTermY = KP_STRAFE * yError;

        double actualTime = goToPosTimer.milliseconds();
        double dTermX = KD_FORWARD * ((xError - previousXError) / (actualTime - previousGoPosTime));
        double dTermY = KD_STRAFE * ((yError - previousYError) / (actualTime - previousGoPosTime));

        double forward = pTermX + dTermX;
        double strafe = pTermY + dTermY;

        Drive(turn, forward, strafe, false);

        previousXError = xError;
        previousYError = yError;
        previousGoPosTime = actualTime;

        return false;
    }
    /**
     * A function that allows the robot to orient itself to a given orientation while moving along x and y axises
     * Return if the robot is oriented yet using tolerance
     * @param headingTarget the orientation that we want the robot to be
     * @param forward the power with which the robot will move forward
     * @param strafe the power with which the robot will move sideway
     * @return if the robot is oriented yet using tolerance (true : yes ; false : no)
     */
    public boolean headToTarget(double headingTarget, double forward, double strafe){

        if (utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING))
            return true;

        double headingError = headingTarget - robotHeading;

        double pTermHeading = KP_TURN * headingError;

        double actualTime = goToPosTimer.milliseconds();
        double dTermHeading = KD_TURN * ((headingError - previousHeadingError)/(actualTime - previousHeadingTime));

        double turn = pTermHeading + dTermHeading;

        Drive(turn, forward, strafe, true);
        previousHeadingError = headingError;
        previousHeadingTime = actualTime;

        return false;
    }

    public boolean goToPosAndHead (double xTarget, double yTarget, double headingTarget){
        if (goToPos(xTarget, yTarget, 1000.0) && headToTarget(headingTarget, 1000.0, 1000.0)){
            return true;
        }
        return false;
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
