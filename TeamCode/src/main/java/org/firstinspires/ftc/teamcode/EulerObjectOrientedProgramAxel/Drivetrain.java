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

    public final int tickPerRevolution = 8592; // Tuneme
    public final double wheelDiameter = 9; //TUNEME
    public final double meterPerTick = (9 * Math.PI) / tickPerRevolution;
    public final double entreAxe = 5;
    public final double entreAxeS = 5;
    public final double kpX = 0.25; //TUNEME
    public final double kpY = 0.25; //TUNEME
    public final double kpHeading = 0.25; //TUNEME
    public final double kdX = 0.25; //TUNEME
    public final double kdY = 0.25; //TUNEME
    public final double kdHeading = 0.25; //TUNEME

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private ElapsedTime PIDTimer = new ElapsedTime();
    private double previousPDError = 0.0;
    private double previousPDTime = 0.0;
    private double previousGoPosTime = 0.0;
    double formerL1 = 0;
    double formerL2 = 0;
    double formerL3 = 0;
    double robotX = 0;
    double robotY = 0;
    double robotHeading = 0;
    // tune all the 3 values above to your robot starting pose
    double previousXError = 0;
    double previousYError = 0;
    double previousHeadingError = 0;

    private Pose2D robotPos;

    public Drivetrain(HardwareMap hmap){
        frontLeftMotor = hmap.get(DcMotor.class, "front left motor");
        frontRightMotor = hmap.get(DcMotor.class, "front right motor");
        backRightMotor = hmap.get(DcMotor.class, "back right motor");
        backLeftMotor = hmap.get(DcMotor.class, "back left motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDTimer.startTime();

    }
    public void Drive (double turn, double forward, double strafe){
        double denominator = Math.max(Math.abs(turn) + Math.abs(forward) + Math.abs(strafe), 1);
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

        double pTermX = kpX * xError;
        double pTermY = kpY * yError;
        double pTermHeading = kpHeading * headingError;

        double actualTime = PIDTimer.milliseconds();
        double dTermX = kdX * ((xError - previousXError)/(actualTime - previousGoPosTime));
        double dTermY = kdY * ((yError - previousYError)/(actualTime - previousGoPosTime));
        double dTermHeading = kdHeading * ((headingError - previousHeadingError)/(actualTime - previousGoPosTime));

        double strafe = pTermX + dTermX;
        double forward = pTermY + dTermY;
        double turn = pTermHeading + dTermHeading;

        Drive(turn, forward, strafe);
    }

    public void AlignWithTarget(double error, double forward, double strafe){

        double pTerm = KP_AUTO_ALIGN * error;

        double actualTime = PIDTimer.milliseconds();
        double dTerm = KD_AUTO_ALIGN * ((error - previousPDError)/(actualTime - previousPDTime));//et il se passe quoi s'il n'y avait pas de previousPDTime ou de previousPDError

        double turn = pTerm + dTerm;

        Drive(turn, forward, strafe);

        previousPDError = error;
        previousPDTime = actualTime;

    }

    public void actualiseRobotPos (){

        double actualL1 = frontLeftMotor.getCurrentPosition() * meterPerTick;
        double actualL2 = frontRightMotor.getCurrentPosition() * meterPerTick;
        double actualL3 = backRightMotor.getCurrentPosition() * meterPerTick;

        double deltaL1 = actualL1 - formerL1;
        double deltaL2 = actualL2 - formerL2;
        double deltaL3 = actualL3 - formerL3;

        double deltaHeading = (deltaL2 - deltaL1)/entreAxe;
        robotHeading += deltaHeading;

        double forward = (deltaL1 + deltaL2)/2;
        double strafe = deltaL3 - deltaHeading * entreAxeS;

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
