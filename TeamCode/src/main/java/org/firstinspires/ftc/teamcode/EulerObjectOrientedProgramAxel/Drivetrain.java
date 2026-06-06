package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private final DcMotor leftMotor; //Je t'ai deja dis, si cette variable est un moteur précise le dans son nom, en plus le mot drive n'est pas vraiment adapté
    private DcMotor rightMotor; //comme pour l'autre moteur
    private double leftMotorPower;
    private double rightMotorPower;
    public enum Drive_mode {
        DRIVE,
        AUTO
    }
    public Drive_mode driveMode = Drive_mode.DRIVE;
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
        leftMotorPower = forward + turn; //je crois qu'il y a une erreur ici
        rightMotorPower = forward - turn; //ici aussi (oublies pas que tu as reverse tes moteurs)    c tt bon mtn

        leftMotor.setPower(leftMotorPower);  //Mettre un voltage compensation sur une base controlee manuelement  est un peu inutile
        rightMotor.setPower(rightMotorPower); //parce qu'on a pas besoin de précision et qu'au contraire ca pourrait la limiter              j'av ta raison

    }

    public void periodic(){

    }

}
