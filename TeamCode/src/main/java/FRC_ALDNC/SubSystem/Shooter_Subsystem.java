package FRC_ALDNC.SubSystem;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.SHOOTER;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKD;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKI;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKP;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.VISEUR;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posviseur_bank;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posviseur_far;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posviseur_mid;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.shooter_aspirage_puissance;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.shooter_velo_tolerance;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.velo_shoot_bank;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.velo_shoot_far;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.velo_shoot_mid;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import lib.Dashboard;
import lib.Utils;


public class Shooter_Subsystem extends SubsystemBase {
    public DcMotorEx shooter;
    public Servo viseur;
    PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
    private static double p = 1400;
    private static double i = 0.0;
    private static double d = 0.0;
    public static double f = 0.0;

    public static double veloShooter = velo_shoot_mid, current_shoot_velo;
    public static double posviseur = posviseur_mid, current_viseur_pos;

    public enum WantedState{
        WAIT,
        SHOOT_BANK,
        SHOOT_MID,
        SHOOT_FAR,
        ASPIRER
    }

    public enum SystemState{
        WAITING,
        PREPARING_TO_SHOOT,
        READY_TO_SHOOT,
        ASPIRER
    }
    private SystemState sysState = SystemState.WAITING;
    private WantedState wantedState = WantedState.WAIT;
    public Shooter_Subsystem (HardwareMap hmap){
        shooter = hmap.get(DcMotorEx.class, SHOOTER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setVelocityPIDFCoefficients(ShooterKP, ShooterKI, ShooterKD, f);

        viseur = hmap.get(Servo.class, VISEUR);
    }
    public void update_input(){
        current_shoot_velo = shooter.getVelocity();
        current_viseur_pos = viseur.getPosition();
    }
    public double getVeloShooter (){return veloShooter;}
    public void setShooter_state (WantedState systemState){this.wantedState = systemState;}
    public SystemState getShooterSysState(){return sysState;}
    public void RunStateShooter(){
        switch (wantedState)
        {
            case WAIT:
                if (sysState != SystemState.WAITING)
                {
                    sysState = SystemState.WAITING;
                }
                break;
            case SHOOT_BANK:
                veloShooter = velo_shoot_bank;
                posviseur = posviseur_bank;
                if (sysState != SystemState.READY_TO_SHOOT)
                {
                    sysState = SystemState.PREPARING_TO_SHOOT;
                }
                break;
            case  SHOOT_MID:
                veloShooter = velo_shoot_mid;
                posviseur = posviseur_mid;
                if (sysState != SystemState.READY_TO_SHOOT)
                {
                    sysState = SystemState.PREPARING_TO_SHOOT;
                }
                break;
            case SHOOT_FAR:
                veloShooter = velo_shoot_far;
                posviseur = posviseur_far;
                if (sysState != SystemState.READY_TO_SHOOT)
                {
                    sysState = SystemState.PREPARING_TO_SHOOT;
                }
                break;
            case ASPIRER:
                if (sysState != SystemState.ASPIRER)
                {
                    sysState = SystemState.ASPIRER;
                }
            default:
                //Dashboard.Telemetry_with_Text("Shooter", "can't run state machine with an unknown wanted state");
                break;
        }

        switch (sysState)
        {
            case WAITING:
                break;

            case PREPARING_TO_SHOOT:
                if (Utils.IsInRange(current_shoot_velo, veloShooter, shooter_velo_tolerance) && current_viseur_pos == posviseur
                )
                {
                    sysState = SystemState.READY_TO_SHOOT;
                }
                break;

            case READY_TO_SHOOT:
                break;
            case ASPIRER:
                break;

            default:
                //Dashboard.Telemetry_with_Text("Shooter", "can't run state machine with an unknown system state");
                break;
        }
    }
    public void Shoot(){
        switch (sysState)
        {
            case WAITING:
                shooter.setVelocity(0);
                break;

            case ASPIRER:
                shooter.setVelocity(shooter_aspirage_puissance);
            case PREPARING_TO_SHOOT:
                shooter.setVelocity(veloShooter);
                viseur.setPosition(posviseur);
                break;
            case READY_TO_SHOOT:
                break;

            default:
                shooter.setVelocity(0);
                Dashboard.Telemetry_with_Text("Shooter", "unknown system state used");
                break;
        }
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

    public void calculate_postir(double distance_to_goal){
        //Definis moi ca
    }
    @Override
    public void periodic(){
        update_input();

        RunStateShooter();
        Shoot();
    }
}
