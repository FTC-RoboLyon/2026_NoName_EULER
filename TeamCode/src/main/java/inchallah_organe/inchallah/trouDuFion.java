package inchallah_organe.inchallah;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class trouDuFion {
    public DcMotor fion;
    public trouDuFion(DcMotor fion){
        this.fion = fion;
        fion.setDirection(DcMotorSimple.Direction.FORWARD);
        fion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public float veloFionBank = 900;
    public float veloFionFar = 2000;
    public float veloFionMid = 1500;
    public float veloFion = veloFionBank;
    public boolean isShooting = false;
    public void caca(boolean b, boolean a, boolean y, boolean rightBumperWpr, double rightTrigger){
        if(b){
            veloFion = veloFionBank;
        }
        if(y){
            veloFion = veloFionMid;
        }
        if(a){
            veloFion = veloFionFar;
        }
        if(rightBumperWpr){
            isShooting = !isShooting;
        }
        if(isShooting) {
            ((DcMotorEx) fion).setVelocity(veloFion);
        }
        if(rightTrigger > 0.2){
            fion.setPower(-0.1);
        }
    }
}
