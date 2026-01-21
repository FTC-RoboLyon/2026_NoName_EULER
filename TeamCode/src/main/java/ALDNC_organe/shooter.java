package ALDNC_organe;

import static ALDNC_organe.Constant.SHOOTER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class shooter {

    final DcMotorEx shooter;
    public static double Velo_bank = 940;
    public static double Velo_Mid = 2000;
    public static double Velo_far = 3000;
    public static double Aspirer = -0.3;
    public static double VeloShooter = Velo_bank;
    public static double kp = 20;
    public static double ki = 5;
    public static double kd = 10;
    public static double kf = 15;

    public static boolean isShooting = false;


    public shooter(HardwareMap hardware){
        shooter = hardware.get(DcMotorEx.class, SHOOTER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }




    public double getpower() {return shooter.getPower();}
    public void setPIDFcoefficient(Gamepad gamepad1, Gamepad gamepad2){
        if (gamepad1.dpad_up) {
            kf += 1;
        }
        if (gamepad1.dpad_down) {
            kf -= 1;
        }
        if (gamepad1.dpad_left) {
            kf -= 0.1;
        }
        if (gamepad1.dpad_right) {
            kf += 0.1;
        }
        if (gamepad2.dpad_up) {
            kp += 1;
        }
        if (gamepad2.dpad_down) {
            kp -= 1;
        }
        if (gamepad2.dpad_left) {
            kp -= 0.1;
        }
        if (gamepad2.dpad_right) {
            kp += 0.1;
        }
        PIDFCoefficients pidf = new PIDFCoefficients(kp, ki, kd, kf);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
    public double getVelocity(){return shooter.getVelocity();}
    public PIDFCoefficients getpidfcoeff(){return shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);}
    public double getVeloShooter(){return VeloShooter;}

    public void setVelo_bank(){shooter.setVelocity(Velo_bank);}
    public void setVelo_mid(){shooter.setVelocity(Velo_Mid);}
    public void setVelo_far(){shooter.setVelocity(Velo_far);}
    public void set_Velo (double veloshoot){shooter.setVelocity(veloshoot);}

    public boolean HaveShoot (){
        return isShooting && shooter.getVelocity() >= VeloShooter - 75;
    } // Cette méthode ne peut être utilisé qui si on est sur que la flywheel a deja attein sa puissance programmé

    public boolean isAtgoodspeed (){
        return Math.abs(shooter.getVelocity() - VeloShooter) <= 10;
    }



    public void regleurVeloShooteur(Gamepad gamepad1, Gamepad gamepad2) {
            if (gamepad2.dpad_up) {
                VeloShooter += 100;
            }
            if (gamepad2.dpad_down) {
                VeloShooter -= 100;
            }
            if (gamepad2.dpad_left) {
                VeloShooter -= 50;
            }
            if (gamepad2.dpad_right) {
                VeloShooter += 50;
            }
            if (gamepad1.bWasPressed()) {
                VeloShooter = Velo_bank;
            }
            if (gamepad1.aWasPressed()) {
                VeloShooter = Velo_Mid;
            }
            if (gamepad1.yWasPressed()) {
                VeloShooter = Velo_far;
            }
            if (gamepad1.right_trigger > 0.1) {
                shooter.setPower(Aspirer);
            }
            if (gamepad1.rightBumperWasPressed()) {
                isShooting = !isShooting;
            }
            shooter.setVelocity(isShooting ? VeloShooter : 0);

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
}
