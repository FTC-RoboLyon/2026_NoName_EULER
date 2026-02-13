package packageClermont.organe;

import static java.lang.Math.pow;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lib.PidRBL;

@Config
public class jambes {

    public DcMotorEx jambe_droite;
    public DcMotorEx jambe_gauche;
    public static double variablePower = 2;
    public static double kp_jambe = 200;
    public static double ki_jambe;
    public static double kd_jambe;
    public static double kf_jambe;
    public static double jambage_tolerance = 0.05;

    public double erreurJG;
    public double erreurJD;
    PidRBL pid = new PidRBL(kp_jambe, ki_jambe, kd_jambe, kf_jambe);

    public jambes (DcMotorEx jambe_droite, DcMotorEx jambe_gauche){
        this.jambe_droite = jambe_droite;
        this.jambe_gauche = jambe_gauche;

        jambe_gauche.setDirection(DcMotorSimple.Direction.REVERSE);
        jambe_droite.setDirection(DcMotorSimple.Direction.FORWARD);
        jambe_droite.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jambe_gauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jambe_gauche.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        jambe_droite.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        jambe_droite.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        jambe_gauche.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pid.SetTolerance(jambage_tolerance);
        pid.SetOutputLimits(-1, 1);
        pid.SetContinuous(false);
    }

    public void jambage(double value_jambeDroite, double value_jambeGauche, Gamepad gamepad1, Telemetry telemetry){
        if(value_jambeDroite >= 0){
            value_jambeDroite = Math.pow(value_jambeDroite, variablePower);
        }else if(value_jambeDroite < 0){
            value_jambeDroite = -Math.pow(value_jambeDroite, variablePower);
        }
        if(value_jambeGauche >= 0){
            value_jambeGauche = Math.pow(value_jambeGauche, variablePower);
        }else if(value_jambeGauche < 0){
            value_jambeGauche = -Math.pow(value_jambeGauche, variablePower);
        }

        jambe_droite.setPower(value_jambeDroite);
        jambe_gauche.setPower(value_jambeGauche);
    }
}
