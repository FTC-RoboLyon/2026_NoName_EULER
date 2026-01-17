package ALDNC_organe;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class shooter {

    final DcMotor shooter;


    public shooter(DcMotor shooter){
        this.shooter = shooter;
        this.shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }


    public void shooter(boolean shoot, double PowerShooter, float left_trigger) {
        if (left_trigger > 0.3){
            shooter.setPower(-0.3);
        }
        shooter.setPower(shoot ? PowerShooter:0);

    }


    public double regleurPuissanceShooter(double velocityShooter, boolean fleche_haut, boolean fleche_bas,
                                          boolean fleche_gauche, boolean fleche_droite, boolean bwp, boolean ywp, boolean awp, double puissance_bank,
                                          double puissance_mid, double puissance_far) {
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
        } if (awp){
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
