package packageClermont.organe;

import static java.lang.Math.pow;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class jambes {

    public DcMotorEx jambe_droite;
    public DcMotorEx jambe_gauche;
    public static double variablePower = 2;
    private static double p = 200;
    private static double i;
    private static double d;
    private static double f;

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
        jambe_droite.setVelocityPIDFCoefficients(p, i, d, f);
        jambe_gauche.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void jambage(double value_jambeDroite, double value_jambeGauche, Gamepad gamepad1){

        if(value_jambeDroite >= 0){
            jambe_droite.setPower(Math.pow(value_jambeDroite, variablePower));
        }else if(value_jambeDroite < 0){
            jambe_droite.setPower(-Math.pow(value_jambeDroite, variablePower));
        }
        if(value_jambeGauche >= 0){
            jambe_gauche.setPower(Math.pow(value_jambeGauche, variablePower));
        }else if(value_jambeGauche < 0){
            jambe_gauche.setPower(-Math.pow(value_jambeGauche, variablePower));
        }
    }
}
