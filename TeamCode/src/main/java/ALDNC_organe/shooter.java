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


    public void shooter(boolean shoot, double PowerShooter, float left_trigger, boolean b_wrp, boolean y_wrp, double Power_bank, double Power_far) {
        if (b_wrp) {
            PowerShooter = Power_bank;
        } else if (y_wrp) {
            PowerShooter = Power_far;
        }
        if (left_trigger > 0.3){
            shooter.setPower(-0.3);
        }
        shooter.setPower(shoot ? PowerShooter:0);

    }


    public double regleurPuissanceShooter(double velocityShooter, boolean fleche_haut, boolean fleche_bas) {
        if (fleche_haut) {
            velocityShooter += 100;
        } else if (fleche_bas) {
            velocityShooter -= 100;
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
