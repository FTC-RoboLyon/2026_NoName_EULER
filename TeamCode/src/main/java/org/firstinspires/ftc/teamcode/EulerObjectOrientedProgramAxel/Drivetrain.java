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
    // tune all the 3 values above to your robot starting pose

    private double previousFwdError = 0;
    private double previousStrafeError = 0;
    private double previousHeadingError = 0;
    private boolean firstGoToPosIteration = true; //stay true until firt iteration is finished
    private boolean firstDriveHeadToTargetIteration = true; //stay true until firt iteration is finished
    // for the two boolean above, stay true until first iteration of their function (become false a this moment)
    // and become true again when target of their function is reached

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

    //ma version des specs :
    /**
     * Allows the drivetrain to move and rotate at given powers.
     * This function handles displacement based on the field or robot axes.
     *
     * @param rotationPower the rotation power given to the robot
     * @param xPower the displacement power along the X axes of the chosen coordinate system
     * @param yPower the displacement power along the Y axes of the chosen coordinate system
     * @param fieldOriented if the power are given in the field coordinate system (if false it assumes that they are given in the robot coordinate system)
     */

    //Le petit pb mtn que j'y pense c'est que les noms fwd et strafe n'ont pas de sens si c'est pas field oriented (XPower et YPower, ou un truc dans le genre serait peut etre mieux)
    //Turn veut d'ailleurs toujours rien dire ce qu'on mesure c'est le "heading" qui varie avec des "rotation"

    //euh d'accord mais ca a du sens de mettre xPower et yPower si c'est field oriented ? fin jveux dire c'est utile que je laisse les variable forward et strafe externe a la fonction comme ca ou faudrait que je travaille uniquement avec xpower et ypower ?
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
     * A function that allows the robot to move to a given point of coordinates (xTarget, yTarget) while turning itself freely.
     * Return if the robot has arrived yet using tolerances.
     * @param xTarget X coordinate of the target point (in meters)
     * @param yTarget Y coordinate of the target point (in meters)
     * @param turn the rotation given directly to the robot (if you don't want the robot to turn just set it to 0)
     *                  For the parameter above, if we give the value of 1000.0, the precedent value given will be kept
     * @return if the robot has arrived yet using tolerances (true : yes; false : no)
     */

    //Je te remets mes specs là :
    /**
     * A function that allows the robot to move to a given point of coordinates (xTarget, yTarget) and head to a given heading target.
     * Return if the robot has arrived yet using tolerances.
     * @param xTarget X coordinate of the target point (in meters)
     * @param yTarget Y coordinate of the target point (in meters)
     * param headingTarget heading target of the robot (in radians)
     * @return if the robot has arrived yet using tolerances (true : yes; false : no)
     */

    //La majeure difference est le heading/turn. Deja le nom heading correspond mieux que turn car turn n'est pas forcement une rotation sur soi-même mais juste tourner
    //ce qui n'a donc aucun sens. Ensuite ca n'a pas de sens de controller les coordonnées X et Y en PID/target mais le heading seulement avec un power pour ce que tu veux faire avec ça.
    // Il faut remettre comme c'etait avant mais tout simplement si tu ne veux pas tourner tu ecrira goToPos(2.0, 0.5, robotHeading).
    //Par contre si tu veux bouger a une coordonnée tout en regardant une target tu auras juste a ecrire goToPos(2.0, 0.5, headToTarget()).
    //Faire comme ca sera beacoup plus propre mais il faut dcp que plutot de controller les moteurs headToTarget ne fasse que calculer la headingTarget
    // donc potentiellement aussi changer son nom et lui faire acceder directement a la camera même si c'est pas obligatoire.
    //Le seul problème est que si tu fais comme je te dis HeadToTarget n'est plus compatible avec un drive power donc il faut creer une nouvelle fonction
    //DriveHeadingHeadingToTarget qui est en fait celle que tu as deja (qui serait d'ailleurs bien plus facilement implémentable avec une machine à état voir directement une logique Subsystem).
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

        Drive(rotationPower, forward, strafe, true);

        previousFwdError = fwdError;
        previousStrafeError = strafeError;
        previousHeadingError = headingError;
        previousGoPosTime = currentTime;

        return false;
    }


   public boolean driveHeadToTarget(double headingTarget, double forward, double strafe){
       if (utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING)){
           firstDriveHeadToTargetIteration = true;
           return true;
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
