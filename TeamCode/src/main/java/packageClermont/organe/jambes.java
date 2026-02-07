package packageClermont.organe;

import static java.lang.Math.pow;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class jambes {

    public DcMotor jambe_droite;
    public DcMotor jambe_gauche;

    public jambes (DcMotor jambe_droite, DcMotor jambe_gauche){
        this.jambe_droite = jambe_droite;
        this.jambe_gauche = jambe_gauche;

        jambe_gauche.setDirection(DcMotorSimple.Direction.REVERSE);
        jambe_droite.setDirection(DcMotorSimple.Direction.FORWARD);
        jambe_droite.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jambe_gauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void jambage(double value_jambeDroite, double value_jambeGauche){
        if(value_jambeDroite >= 0){
            jambe_droite.setPower(Math.pow(value_jambeDroite, 2));
        }else if(value_jambeDroite < 0){
            jambe_droite.setPower(-Math.pow(value_jambeDroite, 2));
        }
        if(value_jambeGauche >= 0){
            jambe_gauche.setPower(Math.pow(value_jambeGauche, 2));
        }else if(value_jambeGauche < 0){
            jambe_gauche.setPower(-Math.pow(value_jambeGauche, 2));
        }
    }
}
