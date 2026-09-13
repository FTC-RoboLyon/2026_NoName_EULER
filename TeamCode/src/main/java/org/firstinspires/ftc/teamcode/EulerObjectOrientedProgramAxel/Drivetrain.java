package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.utils;

public class Drivetrain {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    public final int TICKS_PER_REVOLUTION = 8192 ;
    // Tune this to the number of tick your sensor register per wheel revolution
    public final double WHEEL_RADIUS = 0.45; //TUNEME in meters
    public final double METERS_PER_TICK = (WHEEL_RADIUS * Math.PI * 2) / TICKS_PER_REVOLUTION;
    public final double E = 5.0; // in meters
    public final double ES = 5.0; //in meters, t'as effacé les commentaires mais je continue de dire que l'entraxe de 5 METRES elle ne rentre pas sur un robot FTC (ni même sur un robot FRC) (boh apres a tout moment tu laisses juste ca parce que t'as pas de vraie valeur...)
    public final static double KP_STRAFE = 0.25; //TUNEME
    public final static double KP_FORWARD = 0.25; //TUNEME
    public final static double KP_HEADING = 0.25; //TUNEME
    public final static double KD_STRAFE = 0.25; //TUNEME
    public final static double KD_FORWARD = 0.25; //TUNEME
    public final static double KD_HEADING = 0.25; //TUNEME
    public final static double TOLERANCE_X_AND_Y = 0.05; //TUNEME IN METERS
    public final static double TOLERANCE_HEADING = 0.10; //TUNEME IN RADIANT

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double forward = 0.0, strafe = 0.0;
    private ElapsedTime goToPosTimer = new ElapsedTime();
    private ElapsedTime headingTimer = new ElapsedTime();
    private double previousGoPosTime = 0.0;
    private double previousHeadingTime = 0.0;
    private double previousLeftPodValue = 0;
    private double previousRightPodValue = 0;
    private double previousStrafePodValue = 0;
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;
    // tune all 3 values above to tune your robot starting pos

    private double previousFwdError = 0;
    private double previousStrafeError = 0;
    private double previousHeadingError = 0;
    private boolean firstGoToPosIteration = true; //stay true until first iteration is finished
    private boolean firstDriveHeadToTargetIteration = true; //stay true until first iteration is finished
    // for the two boolean above, stay true until first iteration of their function (become false a this moment)
    // and become true again when target of their function is reached -> eh ben qu'est ce qui te prend de faire autant de commentaire mais bon tant mieux
    // (boh apres je t'avoues cela sont pas très utile le nom de la variable et assez clair mais bon mtn qu'ils sont la autant les laisser)

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
     * Allows the drivetrain to move and rotate at given powers.
     * This function handles displacement based on the field or robot axes.
     *
     * @param rotationPower the rotation power given to the robot
     * @param xPower the displacement power along the X axes of the chosen coordinate system
     * @param yPower the displacement power along the Y axes of the chosen coordinate system
     * @param fieldOriented if the power are given in the field coordinate system (if false it assumes that they are given in the robot coordinate system)
     */

    //euh d'accord mais ca a du sens de mettre xPower et yPower si c'est field oriented ? fin jveux dire c'est utile que je laisse les variable forward et strafe externe a la fonction comme ca ou faudrait que je travaille uniquement avec xpower et ypower ?
    //Pour ta premiere question oui car forward n'est que l'axe X du robot et strafe l'axe Y. Pour ta deuxieme question les variables externes ne te servent plus a r puisqu'elles ne sont utilisées qu'ici donc soit tu les mets en interne soit tu les supprime prc qu'elles servent plus a grand chose
    public void Drive (double rotationPower, double xPower, double yPower, boolean fieldOriented){
        if (fieldOriented){
            forward = Math.cos(robotHeading)*xPower + Math.sin(robotHeading)*yPower;
            strafe = -Math.sin(robotHeading)*xPower + Math.cos(robotHeading)*yPower;
        }else{
            forward = xPower;
            strafe = yPower;
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

    public void actualiseRobotPos (){

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
            firstGoToPosIteration = true;
            return true;
        }


        double xError = xTarget - robotX;
        double yError = yTarget - robotY;
        double headingError = headingTarget - robotHeading;

        double fwdError= Math.cos(robotHeading) * xError + Math.sin(robotHeading) * yError;
        double strafeError = -Math.sin(robotHeading) * xError + Math.cos(robotHeading) * yError;

        double pTermX = KP_FORWARD * fwdError;
        double pTermY = KP_STRAFE * strafeError;
        double pTermHeading = KD_HEADING * headingError;

        double currentTime = goToPosTimer.milliseconds();

        if (firstGoToPosIteration == true){
            previousFwdError = fwdError;
            previousStrafeError = strafeError;
            previousHeadingError = headingError;
            firstGoToPosIteration = false;
        }
        double dTermX = KD_FORWARD * ((fwdError - previousFwdError)/(currentTime - previousGoPosTime));
        double dTermY = KD_STRAFE * ((strafeError - previousStrafeError)/(currentTime - previousGoPosTime));
        double dTermHeading = KD_HEADING * ((headingError - previousHeadingError)/(currentTime - previousGoPosTime));

        double forward = pTermX + dTermX;
        double strafe = pTermY + dTermY;
        double rotationPower = pTermHeading + dTermHeading;

        Drive(rotationPower, forward, strafe, false); //->attention tu as deja passée tout tes calculs dans les coordonées robot

        previousFwdError = fwdError;
        previousStrafeError = strafeError;
        previousHeadingError = headingError;
        previousGoPosTime = currentTime;

        return false;
    }


    //Tu as mis un return boolean mais la je suis pas sur que y'ai besoin parce que même si tu es deja aligné cette fonction est faite pour être utilisée
    //en TeleOp en pilotant avec la manette donc le programme s'en fou de savoir s'il est alginé
    // (en tout cas pour l'instant sauf si tu fais un shoot automatique mais dans ce cas la le return ne doit quand meme pas coupé les inputs de la manette et la correction du heading
   public boolean driveHeadingToTarget(double headingTarget, double forward, double strafe){
       if (utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING)){
           firstDriveHeadToTargetIteration = true;
           return true; //-> le pb c'est que la il coupe avant de donner les consigne de fwd et strafe
       }

       double headingError = headingTarget - robotHeading;

       double pTermHeading = KP_HEADING * headingError;

       if (firstDriveHeadToTargetIteration == true){
           previousHeadingError = headingError;
           firstDriveHeadToTargetIteration = false;
       }

       double actualTime = headingTimer.milliseconds();
       double dTermHeading = KD_HEADING * ((headingError - previousHeadingError)/(actualTime - previousHeadingTime));

       double turn = pTermHeading + dTermHeading;

       Drive(turn, forward, strafe, true);
       previousHeadingError = headingError;
       previousHeadingTime = actualTime;

       return false;
   }

    //public boolean goToPosAndHead (double xTarget, double yTarget, double headingTarget){
    //    if (goToPos(xTarget, yTarget, 1000.0) && headToTarget(headingTarget, 1000.0, 1000.0)){
    //        return true;
    //    }
    //    return false;
    //}

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
