package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {
    public static final double KP_AUTO_ALIGN = 0.5; // TUNEME
    public static final double KD_AUTO_ALIGN = 0.5; // TUNEME

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private double leftMotorPower;
    private double rightMotorPower;
    private ElapsedTime PDTimer = new ElapsedTime();//t'as oublie de le demarrer ton timer
    private static double lastErrorPD;//previous mieux que last et PD doit venir avant Error -> previousPDError
    private static double lastTimePD;//previous mieux que last et PD doit venir avant Time -> previousPDTime
    public Drivetrain(HardwareMap hmap){
        leftMotor = hmap.get(DcMotor.class, "left motor");
        rightMotor = hmap.get(DcMotor.class, "right motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void Drive (double turn, double forward){
        leftMotorPower = forward + turn;//Par convention ca doit etre l'inverse puisque selon les convention un output positif est cense faire tourner dans le sens trogonometrique
        rightMotorPower = forward - turn;//Par convention ca doit etre l'inverse puisque selon les convention un output positif est cense faire tourner dans le sens trogonometrique

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

    }

    public void DriveAlongATarget(double error, double forward){ //le nom de la fonction n'est pas tres fou, ca serait mieux AlignWithTarget ou un truc du genre

        double pTerm = KP_AUTO_ALIGN * error; //choix de nom de variable pas ouf

        double actualTime = PDTimer.milliseconds();
        double dTerm = KD_AUTO_ALIGN * ((error - lastErrorPD)/actualTime - lastTimePD);//et il se passe quoi s'il n'y avait pas de lastTimePD ou de lastErrorPD + il manque des parentheses ;)

        double turn = pTerm + dTerm;

        leftMotorPower = forward + turn; // pareil que ligne 33/34
        rightMotorPower = forward - turn; // pareil que ligne 33/34

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        lastErrorPD = error;
        lastTimePD = actualTime;

    }



}
