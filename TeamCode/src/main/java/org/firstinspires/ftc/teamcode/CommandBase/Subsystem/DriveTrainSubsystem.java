package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Lib.utils;

import java.util.function.DoubleSupplier;

public class DriveTrainSubsystem extends SubsystemBase {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    private ElapsedTime goToPosTimer = new ElapsedTime();
    private ElapsedTime headingTimer = new ElapsedTime();

    private DoubleSupplier ySupplier, xSupplier, turnSupplier;

    public final int TICKS_PER_REVOLUTION = 8192 ;
    // Tune this to the number of tick your sensor register per wheel revolution
    public final double WHEEL_RADIUS = 0.45; //TUNEME in meters
    public final double METERS_PER_TICK = (WHEEL_RADIUS * Math.PI * 2) / TICKS_PER_REVOLUTION;
    public final double E = 5.0; // in meters
    public final double ES = 5.0; //in meters
    public final static double KP_STRAFE = 0.25, KD_STRAFE = 0.25; //TUNEME
    public final static double KP_FORWARD = 0.25, KD_FORWARD = 0.25; //TUNEME
    public final static double KP_HEADING = 0.25, KD_HEADING = 0.25; //TUNEME

    public final static double TOLERANCE_X_AND_Y = 0.05; //TUNEME IN METERS
    public final static double TOLERANCE_HEADING = 0.10; //TUNEME IN RADIANT

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double previousGoPosTime = 0.0;
    private double previousHeadingTime = 0.0;
    private double previousLeftPodValue = 0;
    private double previousRightPodValue = 0;
    private double previousStrafePodValue = 0;
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;
    private double previousFwdError = 0;
    private double previousStrafeError = 0;
    private double previousHeadingError = 0;
    private boolean firstIteration = true; //stay true until first iteration is finished
    // for the two boolean above, stay true until first iteration of their function (become false a this moment)
    // and become true again when target of their function is reached


    private double xTarget = robotX, yTarget = robotY, headingTarget = robotHeading;

    private boolean fieldOriented = true;


    public enum DriveMode{
        IDLE,
        ROBOT_ORIENTED,
        FILED_ORIENTED,
        GO_TO_POS,
        DRIVE_HEADING
    }
    private DriveMode driveMode = DriveMode.IDLE;
    public void setDriveMode(DriveMode drive){
        driveMode = drive;
        if (driveMode == DriveMode.GO_TO_POS || driveMode == DriveMode.DRIVE_HEADING)
            firstIteration = true;
    }
    public DriveMode getDriveMode(){return driveMode;}


    public DriveTrainSubsystem (HardwareMap hmap, DriveMode driveMode,
                                DoubleSupplier ySupplier,
                                DoubleSupplier xSupplier,
                                DoubleSupplier turnSupplier){

        this.ySupplier = ySupplier;
        this.xSupplier = xSupplier;
        this.turnSupplier = turnSupplier;

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

        this.driveMode = driveMode;

        goToPosTimer.startTime();
        goToPosTimer.reset();

        headingTimer.startTime();
        headingTimer.reset();

    }
    public DriveTrainSubsystem (HardwareMap hmap, DriveMode driveMode,
                                DoubleSupplier ySupplier,
                                DoubleSupplier xSupplier,
                                DoubleSupplier turnSupplier,
                                SparkFunOTOS.Pose2D startPos){
        this(hmap, driveMode, ySupplier, xSupplier, turnSupplier);
        robotX = startPos.x;
        robotY = startPos.y;
        robotHeading = startPos.h;
    }
    private void Drive(double rotationPower, double xPower, double yPower){

        double forward = xPower;
        double strafe = yPower;

        if (fieldOriented){
            forward = Math.cos(robotHeading)*xPower + Math.sin(robotHeading)*yPower;
            strafe = -Math.sin(robotHeading)*xPower + Math.cos(robotHeading)*yPower;
        }

        double maxMotorValue = Math.max(Math.abs(rotationPower) + Math.abs(forward) + Math.abs(strafe), 1);

        frontLeftPower = (forward - rotationPower - strafe) / maxMotorValue;
        frontRightPower = (forward + rotationPower + strafe) / maxMotorValue;
        backLeftPower = (forward - rotationPower + strafe) / maxMotorValue;
        backRightPower = (forward + rotationPower - strafe) / maxMotorValue;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    private void actualiseRobotPos(){

        double leftPodValue = frontLeftMotor.getCurrentPosition() * METERS_PER_TICK;
        double rightPodValue = frontRightMotor.getCurrentPosition() * METERS_PER_TICK;
        double strafePodValue = backRightMotor.getCurrentPosition() * METERS_PER_TICK;

        double dLeftValue = leftPodValue - previousLeftPodValue;
        double dRightValue = rightPodValue - previousRightPodValue;
        double dStrafeValue = strafePodValue - previousStrafePodValue;

        double dHeading = (dRightValue - dLeftValue)/ E;
        robotHeading += dHeading;

        double forward = (dLeftValue + dRightValue)/2;
        double strafe = dStrafeValue - dHeading * ES;

        double deltaX = Math.cos(robotHeading)*forward - Math.sin(robotHeading)*strafe;
        double deltaY = Math.sin(robotHeading)*forward + Math.cos(robotHeading)*strafe;

        robotX += deltaX;
        robotY += deltaY;

        previousLeftPodValue = leftPodValue;
        previousRightPodValue = rightPodValue;
        previousStrafePodValue = strafePodValue;
    }

    /**
     * A function that allows the robot to move to a given point of coordinates (xTarget, yTarget) and head to a given heading target.
     * Return if the robot has arrived yet using tolerances.
     * @param Xtarget X coordinate of the target point (in meters)
     * @param Ytarget Y coordinate of the target point (in meters)
     * @param Headingtarget heading target of the robot (in radians)
     * @return if the robot has arrived yet using tolerances (true : yes; false : no)
     */
    public boolean goToPos (double Xtarget, double Ytarget, double Headingtarget) {
        xTarget = Xtarget;
        yTarget = Ytarget;
        headingTarget = Headingtarget;

        if (utils.IsInRange(robotX, xTarget, TOLERANCE_X_AND_Y)
                && utils.IsInRange(robotY, yTarget, TOLERANCE_X_AND_Y)
                && utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING))
        {
            firstIteration = true;
            driveMode = DriveMode.IDLE;
            return true;

        } else {
            driveMode = DriveMode.GO_TO_POS;
            return false;
        }
    }

    public boolean driveHeadingToTarget(double headingTarget){
        this.headingTarget = headingTarget;
        if (utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING)){
            firstIteration = true;
            return true;
        }
        return false;
    }

    public void stopTheRobot(){
        driveMode = DriveMode.IDLE;
        firstIteration = true;
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

    @Override
    public void periodic(){

        actualiseRobotPos();

        switch (driveMode){
            case IDLE:
                Drive(0,0,0);
                break;

            case ROBOT_ORIENTED:
                fieldOriented = false;
                Drive(turnSupplier.getAsDouble(), xSupplier.getAsDouble(), ySupplier.getAsDouble());
                break;

            case FILED_ORIENTED:
                Drive(turnSupplier.getAsDouble(), xSupplier.getAsDouble(), ySupplier.getAsDouble());
                break;

            case GO_TO_POS:

                fieldOriented = false;

                double xError = xTarget - robotX;
                double yError = yTarget - robotY;
                double headingError = headingTarget - robotHeading;

                double fwdError= Math.cos(robotHeading) * xError + Math.sin(robotHeading) * yError;
                double strafeError = -Math.sin(robotHeading) * xError + Math.cos(robotHeading) * yError;

                double pTermX = KP_FORWARD * fwdError;
                double pTermY = KP_STRAFE * strafeError;
                double pTermHeading1 = KD_HEADING * headingError;

                double currentTime = goToPosTimer.milliseconds();

                if (firstIteration){
                    previousFwdError = fwdError;
                    previousStrafeError = strafeError;
                    previousHeadingError = headingError;
                    firstIteration = false;
                }
                double dTermX = KD_FORWARD * ((fwdError - previousFwdError)/(currentTime - previousGoPosTime));
                double dTermY = KD_STRAFE * ((strafeError - previousStrafeError)/(currentTime - previousGoPosTime));
                double dTermHeading1 = KD_HEADING * ((headingError - previousHeadingError)/(currentTime - previousGoPosTime));

                double forward = pTermX + dTermX;
                double strafe = pTermY + dTermY;
                double rotationPower = pTermHeading1 + dTermHeading1;

                Drive(rotationPower, forward, strafe);


                previousFwdError = fwdError;
                previousStrafeError = strafeError;
                previousHeadingError = headingError;
                previousGoPosTime = currentTime;

                break;

            case DRIVE_HEADING:

                fieldOriented = true;

                headingError = headingTarget - robotHeading;

                double pTermHeading2 = KP_HEADING * headingError;

                if (firstIteration){
                    previousHeadingError = headingError;
                    firstIteration = false;
                }

                double actualTime = headingTimer.milliseconds();
                double dTermHeading2 = KD_HEADING * ((headingError - previousHeadingError)/(actualTime - previousHeadingTime));

                double turn = pTermHeading2 + dTermHeading2;

                Drive(turn, xSupplier.getAsDouble(), ySupplier.getAsDouble());
                previousHeadingError = headingError;
                previousHeadingTime = actualTime;

                break;

            default:
                driveMode = DriveMode.IDLE;
                break;

        }
    }
}
