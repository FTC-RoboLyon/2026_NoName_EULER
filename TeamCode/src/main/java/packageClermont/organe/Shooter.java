package packageClermont.organe;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public class Shooter {
    public DcMotorEx shooter;
    public Shooter(DcMotorEx shooter){
        this.shooter = shooter;
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

    }

    PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
    public static double p = 20.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;
    private double powerShooter = 0.0;
    private double powerShooterBank = 0.0;
    private double powerShooterMid = 0.0;
    private double powerShooterFar = 0.0;
    //private float veloShooterBank = 2300;
    private float veloShooterBank = 900;
    //private float veloShooterFar = 3600;
    private float veloShooterFar = 1400;
    //private float veloShooterMid = 2900;
    private float veloShooterMid = 1200;
    private float veloShooter = veloShooterBank;
    double realVelo;
    double erreur;
    public void Tir(boolean b,
                     boolean a,
                     boolean y,
                     boolean isShooting,
                     double rightTrigger,
                     boolean DpadRight,
                     boolean DpadLeft,
                     boolean up,
                     boolean down){
        realVelo = shooter.getVelocity();
        erreur = veloShooter - realVelo;

        if(b){
            veloShooter = veloShooterBank;
        }else if(y){
            veloShooter = veloShooterMid;
        }
        else if(a){
            veloShooter = veloShooterFar;
        }
        if(DpadRight){
            veloShooter = veloShooter + 100;
        }else if (DpadLeft){
            veloShooter -= 100;
        }else if (up){
            veloShooter += 50;
        }else if (down){
            veloShooter -= 50;
        }
        if(isShooting) {
            shooter.setVelocity(veloShooter + p*erreur);
        } else if(rightTrigger > 0.2){
            shooter.setPower(-0.3);
        }else{
            shooter.setPower(0);
        }
        updatePID();



    }
    public double powerShooter(boolean aWpr, boolean bWpr, boolean yWpr, boolean xWpr){
        if(aWpr){
            powerShooter = powerShooter + 0.1;
        }else if(bWpr) {
            powerShooter = powerShooter - 0.1;
        }else if(yWpr){
            powerShooter = powerShooter + 0.05;
        }else if(xWpr){
            powerShooter = powerShooter - 0.05;
        }
        return powerShooter;
    }

    public float veloShooter (boolean b,
                     boolean a,
                     boolean y,
                     boolean DpadRightWpr,
                     boolean DpadLeftWpr,
                     boolean upWpr,
                     boolean downWpr){
        if(b){
            veloShooter = veloShooterBank;
        }else if(y){
            veloShooter = veloShooterMid;
        }
        else if(a){
            veloShooter = veloShooterFar;
        }
        if(DpadRightWpr){
            veloShooter = veloShooter + 100;
        }else if (DpadLeftWpr){
            veloShooter -= 100;
        }else if (upWpr){
            veloShooter += 50;
        }else if (downWpr){
            veloShooter -= 50;
        }
        return veloShooter;
    }
    public void updatePID(){
        //pidf = new PIDFCoefficients(p, i, d, f);
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public double p(boolean deux_aWpr,
                    boolean deux_bWpr,
                    boolean deux_yWpr,
                    boolean deux_xWpr) {
        if (deux_aWpr) {
            p = p + 0.5;
        } else if (deux_bWpr) {
            p = p + 1;
        } else if (deux_yWpr) {
            p = p - 0.5;
        } else if (deux_xWpr) {
            p = p - 1;
        }
        return p;
    }

}
