package org.firstinspires.ftc.teamcode.CommandBase.Subsystem;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.CommandBase.robotContainer;
import org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics.Pose2d;
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
    private robotContainer robot;

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
    private double robotX = 0; //in meters
    private double robotY = 0; //in meters
    private double robotHeading = 0; //in radiants
    private double previousFwdError = 0;
    private double previousStrafeError = 0;
    private double previousHeadingError = 0;
    private boolean PDfirstIteration = true; //stay true until first iteration is finished
    // for the boolean above, stay true until first iteration of goToPos() or driveHeading() (become false a this moment)
    // and become true again when target of the function is reached
    //-> j'en vois qu'une perso et dans ce cas la precise firstIteration de quoi genre PDFFirstIteration


    private double xTarget = robotX, yTarget = robotY, headingTarget = robotHeading;

    private boolean fieldOriented = true;

    private double xPower = 0.0, yPower = 0.0, rotationPower = 0.0;



    public enum DriveMode{
        DISABLE,
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        GO_TO_POS,
        DRIVE_AND_HEAD_TO_TARGET
    }
    private DriveMode driveMode = DriveMode.DISABLE;
    public void setDriveMode(DriveMode drive){
        driveMode = drive;
    }
    public DriveMode getDriveMode(){return driveMode;}


    public DriveTrainSubsystem (HardwareMap hmap,
                                robotContainer robot
                                ){

        if (ySupplier == null)
            ySupplier = ()->0.0;

        if (xSupplier == null)
            xSupplier = ()->0.0;

        if (turnSupplier == null)
            turnSupplier = ()->0.0;

        this.robot = robot;

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
        goToPosTimer.reset();

        headingTimer.startTime();
        headingTimer.reset();

    }

    public DriveTrainSubsystem (HardwareMap hmap,
                                robotContainer robot,
                                Pose2d startPos){
        this(hmap, robot);
        robotX = startPos.xMeters;
        robotY = startPos.yMeters;
        robotHeading = startPos.headingRadians;
    }


    public void setSupplier(DoubleSupplier ySupplier,
                            DoubleSupplier xSupplier,
                            DoubleSupplier turnSupplier){
        this.ySupplier = ySupplier;
        this.xSupplier = xSupplier;
        this.turnSupplier = turnSupplier;
    }

    public void setPose(Pose2d startPos){
        robotX = startPos.xMeters;
        robotY = startPos.yMeters;
        robotHeading = startPos.headingRadians;
    }

    /**
     * A function that allows the robot to move to a given point of coordinates (xTarget, yTarget) and head to a given heading target.
     * Return if the robot has arrived yet using tolerances.
     * @param Xtarget X coordinate of the target point (in meters)
     * @param Ytarget Y coordinate of the target point (in meters)
     * @param Headingtarget heading target of the robot (in radians)
     * @return if the robot has arrived yet using tolerances (true : yes; false : no)
     */
    //Normalement aucune autre fonction n'est censée changer le drive Mode que SetDriveMode même si elles changent les parametres d'un certain drive mode
    public void setGoToPosTargets (double Xtarget, double Ytarget, double Headingtarget) {
        xTarget = Xtarget;
        yTarget = Ytarget;
        headingTarget = Headingtarget;

        PDfirstIteration = true;
    }

    public void setHeadingTarget(double headingTarget){
        this.headingTarget = headingTarget;

        PDfirstIteration = true;
    }

    public boolean isAtXYTargets(){
        return utils.IsInRange(robotX, xTarget, TOLERANCE_X_AND_Y)
                && utils.IsInRange(robotY, yTarget, TOLERANCE_X_AND_Y);
    }

    public boolean isAtHeadingTarget(){
        return utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING);
    }

    public double getDstanceToAPoint(Pose2d point){
        return Math.sqrt( Math.pow(point.xMeters - robotX, 2) + Math.pow(point.yMeters - robotY, 2));
    }


    //pk toutes les fonctions comme ca elles existent encore si tu les utilise pas étant donné qu'elles sont implémentées autrement

    public void stopTheRobot(){
        driveMode = DriveMode.DISABLE;
        PDfirstIteration = true;
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

    private void applyMotorsPower(){

        double maxMotorValue = Math.max(Math.abs(rotationPower) + Math.abs(xPower) + Math.abs(yPower), 1);

        frontLeftPower = (xPower - rotationPower - yPower) / maxMotorValue;
        frontRightPower = (xPower + rotationPower + yPower) / maxMotorValue;
        backLeftPower = (xPower - rotationPower + yPower) / maxMotorValue;
        backRightPower = (xPower + rotationPower - yPower) / maxMotorValue;

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

        double forward = (dLeftValue + dRightValue)/2;
        double strafe = dStrafeValue - dHeading * ES;

        double deltaX = Math.cos(robotHeading)*forward - Math.sin(robotHeading)*strafe;
        double deltaY = Math.sin(robotHeading)*forward + Math.cos(robotHeading)*strafe;

        robotX += deltaX;
        robotY += deltaY;
        robotHeading += dHeading;

        if (robotHeading > 2 * Math.PI){
            robotHeading -= 2 * Math.PI;
        }
        else if (robotHeading < 2 * Math.PI){

        }

        previousLeftPodValue = leftPodValue;
        previousRightPodValue = rightPodValue;
        previousStrafePodValue = strafePodValue;
    }



    @Override
    public void periodic(){

        actualiseRobotPos();
        double headingError = headingTarget - robotHeading;

        switch (driveMode){
            case DISABLE:
                xPower = 0.0;
                yPower = 0.0;
                rotationPower = 0.0;
                break;

            case ROBOT_CENTRIC:
                xPower = xSupplier.getAsDouble();
                yPower = ySupplier.getAsDouble();
                rotationPower = turnSupplier.getAsDouble();

                break;

            case FIELD_CENTRIC:

                xPower = xSupplier.getAsDouble();
                yPower = ySupplier.getAsDouble();
                rotationPower = turnSupplier.getAsDouble();

                double xPower1 = xPower;
                xPower = Math.cos(robotHeading)* xPower1 + Math.sin(robotHeading)* yPower;
                yPower = -Math.sin(robotHeading)*xPower1 + Math.cos(robotHeading)* yPower;

                break;

            case GO_TO_POS:

                double xError = xTarget - robotX;
                double yError = yTarget - robotY;

                double fwdError= Math.cos(robotHeading) * xError + Math.sin(robotHeading) * yError;
                double strafeError = -Math.sin(robotHeading) * xError + Math.cos(robotHeading) * yError;

                double pTermX = KP_FORWARD * fwdError;
                double pTermY = KP_STRAFE * strafeError;
                double pTermHeading1 = KD_HEADING * headingError;

                double currentTime = goToPosTimer.milliseconds();

                if (PDfirstIteration){
                    previousFwdError = fwdError;
                    previousStrafeError = strafeError;
                    previousHeadingError = headingError;
                    PDfirstIteration = false;
                }
                double dTermX = KD_FORWARD * ((fwdError - previousFwdError)/(currentTime - previousGoPosTime));
                double dTermY = KD_STRAFE * ((strafeError - previousStrafeError)/(currentTime - previousGoPosTime));
                double dTermHeading1 = KD_HEADING * ((headingError - previousHeadingError)/(currentTime - previousGoPosTime));

                xPower = pTermX + dTermX;
                yPower = pTermY + dTermY;
                rotationPower = pTermHeading1 + dTermHeading1;

                previousFwdError = fwdError;
                previousStrafeError = strafeError;
                previousHeadingError = headingError;
                previousGoPosTime = currentTime;

                break;

            case DRIVE_AND_HEAD_TO_TARGET:

                double pTermHeading2 = KP_HEADING * headingError;

                if (PDfirstIteration){
                    previousHeadingError = headingError;
                    PDfirstIteration = false;
                }

                double actualTime = headingTimer.milliseconds();
                double dTermHeading2 = KD_HEADING * ((headingError - previousHeadingError)/(actualTime - previousHeadingTime));

                xPower = xSupplier.getAsDouble();
                yPower = ySupplier.getAsDouble();
                rotationPower = pTermHeading2 + dTermHeading2;

                double xPower2 = xPower;
                xPower = Math.cos(robotHeading)* xPower2 + Math.sin(robotHeading)* yPower;
                yPower = -Math.sin(robotHeading)* xPower2 + Math.cos(robotHeading)* yPower;

                previousHeadingError = headingError;
                previousHeadingTime = actualTime;

                break;

            default:
                driveMode = DriveMode.DISABLE;
                robot.getTelemetry().addLine("Please enter a valid drivetrain driveMode");
                break;

        }

        applyMotorsPower();
    }
}
