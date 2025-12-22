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

    public void shooter(double velocityShooter, boolean right_bumper, double right_Trig, double shoot_velo) {
        if (right_bumper) {
            ((DcMotorEx) shooter).setVelocity(velocityShooter);
        } else if (right_Trig > 0.3) {
            ((DcMotorEx) shooter).setVelocity(-500);
        } else {
            ((DcMotorEx) shooter).setVelocity(0);
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

}
