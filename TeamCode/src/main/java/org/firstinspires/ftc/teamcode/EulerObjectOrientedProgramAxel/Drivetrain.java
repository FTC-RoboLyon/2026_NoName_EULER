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
    public final static double KP_TURN = 0.25; //TUNEME, heading ca peut etre mieux que turn qui par sons sens pourrait inclure un mouvement sur les axes X et Y
    public final static double KD_STRAFE = 0.25; //TUNEME
    public final static double KD_FORWARD = 0.25; //TUNEME
    public final static double KD_TURN = 0.25; //TUNEME, same
    public final static double TOLERANCE_X_AND_Y = 0.05; //TUNEME IN METERS
    public final static double TOLERANCE_HEADING = 0.25; //TUNEME IN RADIANT, 0.25 rad = 14 deg, je veux bien qu'on soit tolerants mais ca fait bcp la


    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double forward = 0.0, strafe = 0.0, turn = 0.0;
    private ElapsedTime goToPosTimer = new ElapsedTime();
    private ElapsedTime headingTimer = new ElapsedTime();
    private double previousGoPosTime = 0.0;
    private double previousHeadingTime = 0.0;
    double previousLeftPodValue = 0;
    double previousRightPodValue = 0;
    double previousStrafePodValue = 0;
    double robotX = 0;
    double robotY = 0;
    double robotHeading = 0;
    // tune all the 3 values above to your robot starting pose TUNEME
    double previousFwdError = 0;
    double previousStrafeError = 0;
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

    //ma version des specs :
    /**
     * Allows the drivetrain to move and rotate at given powers.
     * This function handles displacement based on the field or robot axes.
     *
     * @param Turn the rotation power given to the robot
     * @param Forward the displacement power along the X axes of the chosen coordinate system
     * @param Strafe the displacement power along the Y axes of the chosen coordinate system
     * @param fieldOriented if the power are given in the field coordinate system (if false it assumes that they are given in the robot coordinate system)
     */

    //Le petit pb mtn que j'y pense c'est que les noms fwd et strafe n'ont pas de sens si c'est pas field oriented (XPower et YPower, ou un truc dans le genre serait peut etre mieux)
    //Turn veut d'ailleurs toujours rien dire ce qu'on mesure c'est le "heading" qui varie avec des "rotation"
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
    public boolean goToPos (double xTarget, double yTarget, double turn) {
        //return true if the robot is already at the giving target point and heading
        if (utils.IsInRange(robotX, xTarget, TOLERANCE_X_AND_Y)
                && utils.IsInRange(robotY, yTarget, TOLERANCE_X_AND_Y)) {
            return true;
        }

        double xError = xTarget - robotX;
        double yError = yTarget - robotY;

        double fwdError= Math.cos(robotHeading) * xError + Math.sin(robotHeading) * yError;
        double strafeError = -Math.sin(robotHeading) * xError + Math.cos(robotHeading) * yError;

        double pTermFwd = KP_FORWARD * fwdError;
        double pTermStrafe = KP_STRAFE * strafeError;

        double actualTime = goToPosTimer.milliseconds(); //actual veut toujours dire réel, maintenaint c'est current
        double dTermFwd = KD_FORWARD * ((fwdError - previousFwdError) / (actualTime - previousGoPosTime));
        double dTermStrafe = KD_STRAFE * ((strafeError - previousStrafeError) / (actualTime - previousGoPosTime));
        //encore une fois a ta premiere boucle ou au changement de target la valeur de tes previous risque d'être totalament erronée et causer un overshoot
        //Example : imagine previous est encore a 0, ta vraie position est 0 et tu veux aller a 10. Ta currentError sera donc de 10 et ton dTerm calculera KD*(10-0)/(on va dire 0.002 pour l'exemple) = KD*5000
        // ce qui est beaucoup trop et cause l'overshoot et encore la j'ai fait comme si tu eviter l'overshoot sur le GoPosTime

        double forward = pTermFwd + dTermFwd;
        double strafe = pTermStrafe + dTermStrafe;

        Drive(turn, forward, strafe, false);

        previousFwdError = fwdError;
        previousStrafeError = strafeError;
        previousGoPosTime = actualTime;

        return false;
    }

    /**
     * A function that allows the robot to orient itself to a given orientation while moving along x and y axes -> orient c pas fou, c'est mieux head
     * Return if the robot is oriented yet using tolerance -> oriented to what
     * @param headingTarget the orientation that we want the robot to be -> pourquoi se compliquer la vie
     * @param forward the power with which the robot will move forward
     * @param strafe the power with which the robot will move sideway
     * @return if the robot is heading to the target using tolerance (true : yes ; false : no)
     */

    //Ma version corrigée des specs :
    /**
     * A function that allows the robot to head to a given headingTarget while moving
     * Return if the robot is heading to the target using tolerance
     * @param headingTarget the target heading of the robot
     * @param forward the forward power applied to the robot
     * @param strafe the strafe power applied to the robot
     * @return if the robot has reach its heading target using tolerance (true : yes ; false : no)
     */
    public boolean headToTarget(double headingTarget, double forward, double strafe){

        if (utils.IsInRange(robotHeading, headingTarget, TOLERANCE_HEADING))
            return true;

        double headingError = headingTarget - robotHeading;

        double pTermHeading = KP_TURN * headingError;

        double actualTime = goToPosTimer.milliseconds();
        double dTermHeading = KD_TURN * ((headingError - previousHeadingError)/(actualTime - previousHeadingTime));
        //tjr le meme pb d'overshoot

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
