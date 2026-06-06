package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrain {
    private DcMotor left_drive; //Je t'ai deja dis, si cette variable est un moteur précise le dans son nom, en plus le mot drive n'est pas vraiment adapté
    private DcMotor right_drive; //comme pour l'autre moteur
    public double left_motor_power;
    public double right_motor_power;
    public enum Drive_mode {
        DRIVE,
        AUTO
    }
    public Drive_mode driveMode = Drive_mode.DRIVE;
    public drivetrain(HardwareMap hmap){
        left_drive = hmap.get(DcMotor.class, "left motor");
        right_drive = hmap.get(DcMotor.class, "right motor");

        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setDirection(DcMotorSimple.Direction.FORWARD);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void Drive (double turn, double forward){
        left_motor_power = forward + turn; //je crois qu'il y a une erreur ici
        right_motor_power = forward - turn; //ici aussi (oublies pas que tu as reverse tes moteurs)    c tt bon mtn

        left_drive.setPower(left_motor_power);  //Mettre un voltage compensation sur une base controlee manuelement  est un peu inutile
        right_drive.setPower(right_motor_power); //parce qu'on a pas besoin de précision et qu'au contraire ca pourrait la limiter              j'av ta raison

    }

    public void periodic(){

    }

}
