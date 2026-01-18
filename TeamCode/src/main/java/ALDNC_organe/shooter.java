package ALDNC_organe;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class shooter {

    final DcMotorEx shooter;
    static double Power_bank = 0.41;
    static double powerMid = 0.56;
    static double Power_far = 0.7;
    static double PowerShooter = Power_bank;


    public shooter(DcMotorEx shooter){
        this.shooter = shooter;
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void Shooter(boolean shoot, double PowerShooter, boolean left_trigger) {
        if (left_trigger){
            shooter.setPower(-0.3);
        }
        shooter.setPower(shoot ? PowerShooter:0);

    }
    public double getpower() {return shooter.getPower();}


    public double regleurPuissanceShooter(double velocityShooter, boolean fleche_haut, boolean fleche_bas,
                                          boolean fleche_gauche, boolean fleche_droite, boolean bwp, boolean awp, boolean ywp, double puissance_bank, double puissance_mid, double puissance_far) {
        if (fleche_haut) {
            velocityShooter += 0.01;
        } if (fleche_bas) {
            velocityShooter -= 0.01;
        } if (fleche_gauche) {
            velocityShooter -= 0.01;
        } if (fleche_droite) {
            velocityShooter += 0.01;
        } if (bwp) {
            velocityShooter = puissance_bank;
        } if (ywp) {
            velocityShooter = puissance_mid;
        }
        if (awp){
            velocityShooter = puissance_far;
        }

        return velocityShooter;
    }

    public int compteurBalles(int nbeBallesInsideBot, boolean v, boolean x){
        if(v && x){
            nbeBallesInsideBot -= 1;
        }
        if(nbeBallesInsideBot < 0){
            nbeBallesInsideBot = 0;
        }
        return nbeBallesInsideBot;
    }

    public double getPower (){
        return shooter.getPower();
    }

}
