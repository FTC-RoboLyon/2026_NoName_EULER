package FRC_ALDNC.SubSystem;

import static FRC_ALDNC.Constant.SHOOTER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import packageClermont.organe.Shooter;
@Config
public class Shooter_Subsystem extends SubsystemBase {
    public DcMotorEx shooter;
    PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
    public static double p = 20.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    public static double powerShooterBank = 0.0;
    public static double powerShooterMid = 0.0;
    public static double powerShooterFar = 0.0;
    public static double powerShooter = powerShooterBank;

    public static float veloShooterBank = 900;
    public static float veloShooterFar = 1400;
    public static float veloShooterMid = 1200;
    public static float veloShooter = veloShooterBank;

    double realVelo;
    double erreur;
    public Shooter_Subsystem (HardwareMap hmap){
        shooter = hmap.get(DcMotorEx.class, SHOOTER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public void Tir_using_velo(boolean isShooting, Gamepad gamepad){
        if(isShooting) {
            shooter.setVelocity(veloShooter);
        } else if(gamepad.right_trigger > 0.2){
            shooter.setPower(-0.3);
        }else{
            shooter.setPower(0);
        }
    }
    public void updatePID(){
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public float veloShooter (Gamepad gamepad1){
        if(gamepad1.bWasPressed()){
            veloShooter = veloShooterBank;
        }else if(gamepad1.yWasPressed()){
            veloShooter = veloShooterMid;
        }
        else if(gamepad1.aWasPressed()){
            veloShooter = veloShooterFar;
        }
        return veloShooter;
    }
    @Override
    public void periodic(){
        updatePID();

    }
}
