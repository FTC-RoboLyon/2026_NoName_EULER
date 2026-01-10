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

    public void shooter(double PowerShooter, double right_Trig, boolean shoot) {

        if (right_Trig > 0.3) {
            shooter.setPower(-0.3);
        }
        if (shoot){
            shooter.setPower(PowerShooter);
        }else if (!shoot){
            shooter.setPower(0);
        }

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
