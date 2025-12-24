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

    public void shooter(double PowerShooter, boolean right_bumperWR, double right_Trig, double shoot_velo, int v) {

        if (right_bumperWR) {
            v = -v;
        } else if (right_Trig > 0.3 || shoot_velo > 0) {
            ((DcMotorEx) shooter).setPower(-0.3);
        }
        if (v == 1){
            ((DcMotorEx) shooter).setPower(PowerShooter);
        }else if (v == -1){
            ((DcMotorEx) shooter).setPower(0);
        }
        return v;
    }


    public double regleurPuissanceShooter(double velocityShooter, boolean fleche_haut, boolean fleche_bas) {
        if (fleche_haut) {
            velocityShooter += 100;
        } else if (fleche_bas) {
            velocityShooter -= 100;
        }
        return velocityShooter;
    }

    public int compteurBalles(int nbeBallesInsideBot, int v, boolean x){
        if(v == 1 && x){
            nbeBallesInsideBot -= 1;
        }
        if(nbeBallesInsideBot < 0){
            nbeBallesInsideBot = 0;
        }
        return nbeBallesInsideBot;
    }

}
