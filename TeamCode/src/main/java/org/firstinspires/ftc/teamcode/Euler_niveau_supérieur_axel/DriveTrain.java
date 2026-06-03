package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveTrain {
    private DcMotor left_drive; //Je t'ai deja dis, si cette variable est un moteur précise le dans son nom, en plus le mot drive n'est pas vraiment adapté
    private DcMotor right_drive; //comme pour l'autre moteur
    public double left_motor_power;
    public double right_motor_power;
    public enum Drive_mode {
        DRIVE,
        AUTO
    }
    public Drive_mode driveMode = Drive_mode.DRIVE;
    public DriveTrain (HardwareMap hmap){
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
        left_motor_power = turn - forward; //je crois qu'il y a une erreur ici
        right_motor_power = turn + forward; //ici aussi (oublies pas que tu as reverse tes moteurs)
    }

    public void loop(double voltage){
        left_drive.setPower(left_motor_power*voltage/13);  //Mettre un voltage compensation sur une base controlee manuelement  est un peu inutile
        right_drive.setPower(right_motor_power*voltage/13); //parce qu'on a pas besoin de précision et qu'au contraire ca pourrait la limiter

    }

}
