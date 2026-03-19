package FRC_ALDNC.Auto.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FRC_ALDNC.Auto.Command.ShooterCommand;
import FRC_ALDNC.Auto.Constant;

public class ShooterSubsystem extends SubsystemBase {
    HardwareMap hmap;
    DcMotorEx shooter;
    public ShooterSubsystem(HardwareMap hmap){
        this.hmap = hmap;
        shooter = hmap.get(DcMotorEx.class, Constant.SHOOTER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public double p = 1400,i = 0.0, d = 200, f = 0.0;
    private double powerBank = 900, powerMid = 1200, powerFar = 1500, veloShooter;
    private double lastVelocity = 0;
    private int ballCount = 0;
    public void configureShooter(ShooterCommand.shooterState state){
        switch(state) {
            case Bank:
                veloShooter = powerBank;
                break;
            case Mid:
                veloShooter = powerMid;
                break;
            case Far:
                veloShooter = powerFar;
                break;
            case Wait:
                veloShooter = 0;
        }
    }
    public void updateBallCount() {

        double currentVelocity = shooter.getVelocity();
        double delta = lastVelocity - currentVelocity;

        if(delta >= 100){
            ballCount++;
        }

        lastVelocity = currentVelocity;
    }
    public boolean bonneVitesseOuPas(){
        return shooter.getVelocity() - veloShooter <= 20 && shooter.getVelocity()-veloShooter >= 0;
    }
    public int getBallCount(){
        return ballCount;
    }
    @Override
    public void periodic() {
        shooter.setVelocity(veloShooter);
        updateBallCount();
    }
}
