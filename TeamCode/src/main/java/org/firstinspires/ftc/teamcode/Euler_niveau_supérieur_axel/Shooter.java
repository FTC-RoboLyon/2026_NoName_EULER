package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Shooter {
    private DcMotorEx ShooterMotor;
    public double CPR = 28;

    public static double shooterTolerance = 100;
    public static double shooterKp, shooterKv, shooterKs;
    public static double ShooterPower;

    public static double posviseur_bank = 0.3, posviseur_mid = 0.58, posviseur_far = 0.45; //TUNEME
    public static double veloNear = 1250, veloMid = 1500, veloFar = 1500; //TUNEME



    public enum ShooterState{
        VeloShootNear,
        VeloShootMid,
        VeloShootFar,
        Idle;

    }
    private ShooterState shooterState = ShooterState.Idle;
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }


    public Shooter (HardwareMap hmap){
        ShooterMotor = hmap.get(DcMotorEx.class, "shooter");

        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double get_Shooter_RPM (){
        return (ShooterMotor.getVelocity() / CPR) * 60;     // if yous want to use a gear ratio, divide this by this gear ratio
    }
    public double getVoltageCompensated (double power, double voltage){
        return (power*voltage)/13;
    }
    public void SetVeloShooter (double velocity, double voltage){
        double error = velocity-get_Shooter_RPM();
        if (Math.abs(error) < shooterTolerance)
            error = 0;
        shooterKs = getVoltageCompensated(shooterKs, voltage);
        shooterKv = getVoltageCompensated(shooterKv, voltage);
        double ff = (shooterKv*velocity) + shooterKs;
        double fb = error * shooterKp;

        ShooterPower = ff + fb;
    }

    public void shooterLoop(double voltage){
        switch (getShooterState()){
            case Idle:
                SetVeloShooter(0, voltage);
                break;
            case VeloShootNear:
                SetVeloShooter(veloNear, voltage);
                break;
            case VeloShootMid:
                SetVeloShooter(veloMid, voltage);
                break;
            case VeloShootFar:
                SetVeloShooter(veloFar, voltage);
                break;

        }
        ShooterMotor.setPower(getVoltageCompensated(ShooterPower, voltage));
    }
}
