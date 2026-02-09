package packageClermont.organe;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Shooter {
    public DcMotorEx shooter;
    public Shooter(DcMotorEx shooter){
        this.shooter = shooter;
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(p, i, d, f);

    }

    //PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
    public double p = 1400;
    public double i = 0.0;
    public double d = 0.0;
    public double f = 0.0;
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
    public void Tir_using_velo(boolean b,
                     boolean a,
                     boolean y,
                     boolean isShooting,
                     double rightTrigger,
                     boolean DpadRight,
                     boolean DpadLeft,
                     boolean up,
                     boolean down){

        if(isShooting) {
            shooter.setVelocity(veloShooter);
        } else if(rightTrigger > 0.2){
            shooter.setVelocity(-200);
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
        return veloShooter;
    }
    public void updatePID(){
        //pidf = new PIDFCoefficients(p, i, d, f);
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public void p(boolean deux_aWpr,
                    boolean deux_bWpr,
                    boolean deux_yWpr,
                    boolean deux_xWpr,
                    Telemetry tele) {
        if (deux_aWpr) {
            p = p - 5;
            tele.addData("blabla", "1");
        } else if (deux_bWpr) {
            p = p + 5;
            tele.addData("blabla", "2");
        } else if (deux_yWpr) {
            p = p - 10;
            tele.addData("blabla", "3");
        } else if (deux_xWpr) {
            p = p + 10;
            tele.addData("blabla", "4");
        }
        updatePID();
        tele.addData("p", p);
    }

    public void d(Gamepad gamepad2, Telemetry tele){
        if(gamepad2.aWasPressed()){
            d = d+5;
        }else if(gamepad2.yWasPressed()){
            d  = d-5;
        }else if(gamepad2.bWasPressed()){
            d = d+10;
        }else if(gamepad2.xWasPressed()){
            d  = d-10;
        }
        tele.addData("d", d);

    }



}
