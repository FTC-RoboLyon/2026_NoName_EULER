package inchallah_organe.inchallah;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class trouDuFion {
    public DcMotorEx fion;
    public trouDuFion(DcMotorEx fion){
        this.fion = fion;
        fion.setDirection(DcMotorSimple.Direction.FORWARD);
        fion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fion.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fion.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fion.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }
    PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
    public static double p = 10.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;
    public float veloFionBank = 900;
    public float veloFionFar = 2000;
    public float veloFionMid = 1500;
    public float veloFion = veloFionBank;
    public void caca(boolean b,
                     boolean a,
                     boolean y,
                     boolean isShooting,
                     double rightTrigger,
                     boolean DpadRight,
                     boolean DpadLeft,
                     boolean up,
                     boolean down,
                     boolean deux_aWpr,
                     boolean deux_bWpr,
                     boolean deux_yWpr,
                     boolean deux_xWpr){
        if(b){
            veloFion = veloFionBank;
        }else if(y){
            veloFion = veloFionMid;
        }
        else if(a){
            veloFion = veloFionFar;
        }
        if(DpadRight){
            veloFion = veloFion + 100;
        }else if (DpadLeft){
            veloFion -= 100;
        }else if (up){
            veloFion += 50;
        }else if (down){
            veloFion -= 50;
        }
        if(isShooting) {
            fion.setVelocity(veloFion);
        } else if(rightTrigger > 0.2){
            fion.setPower(-0.1);
        }else{
            fion.setPower(0);
        }
        if(deux_aWpr){
            p = p+0.01;
        }else if(deux_bWpr){
            p = p-0.01;
        }else if(deux_yWpr){
            p = p+0.1;
        }else if(deux_xWpr){
            p = p-0.1;
        }
        updatePID();



    }
    public float veloShooter (boolean b,
                     boolean a,
                     boolean y,
                     boolean DpadRightWpr,
                     boolean DpadLeftWpr,
                     boolean upWpr,
                     boolean downWpr){
        if(b){
            veloFion = veloFionBank;
        }else if(y){
            veloFion = veloFionMid;
        }
        else if(a){
            veloFion = veloFionFar;
        }
        if(DpadRightWpr){
            veloFion = veloFion + 100;
        }else if (DpadLeftWpr){
            veloFion -= 100;
        }else if (upWpr){
            veloFion += 50;
        }else if (downWpr){
            veloFion -= 50;
        }
        return veloFion;
    }
    public void updatePID(){
        pidf = new PIDFCoefficients(p, i, d, f);
        fion.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }
}
