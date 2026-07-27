package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drivetrain {
    public static final double KP_AUTO_ALIGN = 0.5; // TUNEME
    public static final double KD_AUTO_ALIGN = 0.5; // TUNEME

    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    public final int TICKS_PER_REVOLUTION = 8592; // Tuneme
    // bizarre comme valeur qui sort de nulle part, tu l'as calculee avec des reduction c'est ca ?
    // Si c'est le cas ajoute une constante gear ratio et laisse la machine faire le calcul (comme pour les stagiaires)
    public final double WHEEL_DIAMETER = 9; //TUNEME
    // il faut preciser l'unite (qu'on prends d'ailleurs de preference dans le SI) et en general on prends plutôt le rayon qui est plus utile
    public final double METERS_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REVOLUTION;
    // verifie tes unites ;)
    public final double ENTRE_AXES = 5.0; //mets juste e et precise l'unite (de prefernece dans le SI) 
    // et c'est un double donc chiffre a virgule pour eviter les erreurs d'arrondis et de conversions on de la machine on precise .0
    public final double ENTRE_AXES_S = 5.0; //mets juste eS et precise l'unite (de prefernece dans le SI)
    public final static double KP_X = 0.25; //TUNEME
    public final static double KP_Y = 0.25; //TUNEME
    public final static double KP_HEADING = 0.25; //TUNEME
    public final static double KD_X = 0.25; //TUNEME
    public final static double KD_Y = 0.25; //TUNEME
    public final static double KD_HEADING = 0.25; //TUNEME


    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private ElapsedTime PIDTimer = new ElapsedTime();
    private double previousPDError = 0.0;
    private double previousPDTime = 0.0;
    private double previousGoPosTime = 0.0;
    double formerL1 = 0; //on part de L dans la theorie mais dans la pratique on parle plutot de Left et Right pour
    // que ca ait un sens (et tu peux meme rajouter value apres)
    // et on dit plutot previous dans ce cas la puisque c'est la suite instantanee
    double formerL2 = 0; //same
    double formerL3 = 0; //same
    double robotX = 0; //good name :)
    double robotY = 0; //good name :)
    double robotHeading = 0; //good name :)
    // tune all the 3 values above to your robot starting pose (also well thought, you can just add a TUNEME mention)
    // You can also add the possibility to set them in the constructor or a function because you don't start in the same pos when you're red or blue
    double previousXError = 0;
    double previousYError = 0;
    double previousHeadingError = 0;

    private Pose2D robotPos; //bonne initiative de le creer mais dcp autant l'utiliser

    public Drivetrain(HardwareMap hmap){
        frontLeftMotor = hmap.get(DcMotor.class, "front left motor");
        frontRightMotor = hmap.get(DcMotor.class, "front right motor");
        backRightMotor = hmap.get(DcMotor.class, "back right motor");
        backLeftMotor = hmap.get(DcMotor.class, "back left motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //You must also reset wheels encoder or save their current value in previousL1, etc.

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Why do you do the init of only two of you 4 drive motors ?

        PIDTimer.startTime();
    }
    public void Drive (double turn, double forward, double strafe){
        double denominator = Math.max(Math.abs(turn) + Math.abs(forward) + Math.abs(strafe), 1);
        //why not it's a good idea but the name is not very clear if you can find sth else ;)
        frontLeftPower = (forward - turn - strafe) / denominator;
        frontRightPower = (forward + turn + strafe) / denominator;
        backLeftPower = (forward - turn + strafe) / denominator;
        backRightPower = (forward + turn - strafe) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void goPos (double xTarget, double yTarget, double headingTarget){
        double xError = xTarget - robotX;
        double yError = yTarget - robotY;
        double headingError = headingTarget - robotHeading;

        double pTermX = KP_X * xError;
        double pTermY = KP_Y * yError;
        double pTermHeading = KP_HEADING * headingError;

        double actualTime = PIDTimer.milliseconds();
        double dTermX = KD_X * ((xError - previousXError)/(actualTime - previousGoPosTime));
        double dTermY = KD_Y * ((yError - previousYError)/(actualTime - previousGoPosTime));
        double dTermHeading = KD_HEADING * ((headingError - previousHeadingError)/(actualTime - previousGoPosTime));

        double strafe = pTermX + dTermX;
        double forward = pTermY + dTermY;
        double turn = pTermHeading + dTermHeading;

        Drive(turn, forward, strafe);
        //Ca ca va pas marcher (par exemple regarde ce qu'il se passe quand tu veux aller a 1,1,180 et que tu as deja 180 de heading surtout au niveau des moteurs)
    }

    public void AlignWithTarget(double error, double forward, double strafe){
        //j'aurais plutot appele ca head to target parce que align sous entend que ce peut affecter les mouvements lateraux

        double pTerm = KP_AUTO_ALIGN * error;

        double actualTime = PIDTimer.milliseconds();
        double dTerm = KD_AUTO_ALIGN * ((error - previousPDError)/(actualTime - previousPDTime));//et il se passe quoi s'il n'y avait pas de previousPDTime ou de previousPDError

        double turn = pTerm + dTerm;

        Drive(turn, forward, strafe);

        previousPDError = error;
        previousPDTime = actualTime;
    }

    public void actualiseRobotPos (){

        double actualL1 = frontLeftMotor.getCurrentPosition() * METERS_PER_TICK; //Alors actual ca veut dire reel donc plutot current ;) (ou même juste RightPodValue)
        double actualL2 = frontRightMotor.getCurrentPosition() * METERS_PER_TICK; //same
        double actualL3 = backRightMotor.getCurrentPosition() * METERS_PER_TICK; //same

        double deltaL1 = actualL1 - formerL1;// pour delta tu peux juste mettre d et apres plutot Right ou Left que L1 et L2 qui veulent rien dire
        double deltaL2 = actualL2 - formerL2;//same
        double deltaL3 = actualL3 - formerL3;//same

        double deltaHeading = (deltaL2 - deltaL1)/ ENTRE_AXES;//same
        robotHeading += deltaHeading;

        double forward = (deltaL1 + deltaL2)/2;
        double strafe = deltaL3 - deltaHeading * ENTRE_AXES_S;

        double deltaX = Math.cos(robotHeading) * forward - Math.sin(robotHeading) * strafe;
        double deltaY = Math.sin(robotHeading) * forward + Math.cos(robotHeading) * strafe;

        robotX += deltaX;
        robotY += deltaY;

        formerL1 = actualL1;
        formerL2 = actualL2;
        formerL3 = actualL3;
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
