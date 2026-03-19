package FRC_ALDNC.SubSystem;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.SHOOTER;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKD;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKD_velo;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKF;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKF_velo;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKI;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKI_velo;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKP;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ShooterKP_velo;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.VISEUR;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posviseur_bank;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posviseur_far;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.posviseur_mid;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.seuil_volt_shooter;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.shooter_aspirage_puissance;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.shooter_velo_tolerance;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.velo_shoot_bank;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.velo_shoot_far;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.velo_shoot_mid;

/*import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;*/
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import FRC_ALDNC.ALDNC_container;
import lib.Dashboard;
import lib.PID_shooter;
import lib.Utils;

@Config
public class Shooter_Subsystem extends SubsystemBase {
    public DcMotorEx shooter;
    public Servo viseur;
    public CRServo pont;

    public Telemetry telemetry;
    public FtcDashboard dashboard;
    PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
    private static double p = 0;
    private static double i = 0.0;
    private static double d = 0.0;
    public static double f = 0.00509493117974126;

    public static double veloShooter = velo_shoot_mid, current_shoot_velo, veloShooter_auto;
    public static double posviseur = posviseur_mid, current_viseur_pos, posviseur_auto;

    public enum WantedState{
        WAIT,
        SHOOT_BANK,
        SHOOT_MID,
        SHOOT_FAR,
        AUTO,
        ASPIRER
    }

    public enum SystemState{
        WAITING,
        PREPARING_TO_SHOOT,
        READY_TO_SHOOT,
        ASPIRER
    }
    VoltageSensor voltageSensor;

    private SystemState sysState = SystemState.WAITING;
    private WantedState wantedState = WantedState.WAIT;
    double voltage = 12.0;
    public static double Pow_shoot;
    public static PID_shooter shooter_pidf;
    public boolean inauto;
    InterpLUT met_la_moi_profond;
    InterpLUT apprend_a_viser;

    private ALDNC_container robot;
    public DoubleSupplier distance_To_goal;

    public Shooter_Subsystem (HardwareMap hmap, Telemetry telemetry, ALDNC_container RoBot, boolean auto, DoubleSupplier distance_to_goal){
        distance_To_goal = distance_to_goal;

        inauto = auto;
        robot = RoBot;
        shooter_pidf = new PID_shooter(ShooterKP, ShooterKI, ShooterKD, ShooterKF);
        shooter_pidf.SetTolerance(shooter_velo_tolerance);

        met_la_moi_profond = new InterpLUT();
        apprend_a_viser = new InterpLUT();

        dashboard = FtcDashboard.getInstance();
        shooter = hmap.get(DcMotorEx.class, SHOOTER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        viseur = hmap.get(Servo.class, VISEUR);
        pont = hmap.get(CRServo.class, "chemin");

        this.telemetry = telemetry;

        voltageSensor = hmap.get(VoltageSensor.class, "Control Hub");

    }
    void apprend_a_viser(){
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);
        met_la_moi_profond.add(0,0);

        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
        apprend_a_viser.add(0,0);
    }
    public void update_input(){
        current_shoot_velo = shooter.getVelocity();
        current_viseur_pos = viseur.getPosition();
        voltage = voltageSensor.getVoltage();
    }
    public boolean Is_Shooting(){return sysState == SystemState.PREPARING_TO_SHOOT || sysState == SystemState.READY_TO_SHOOT;}
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
                if (inauto){
                    veloShooter = velo_shoot_mid;
                }else {
                veloShooter = velo_shoot_mid;}
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

            case AUTO:
                calculate_postir(distance_To_goal.getAsDouble());
                veloShooter = veloShooter_auto;
                posviseur = posviseur_auto;
                if (sysState != SystemState.READY_TO_SHOOT)
                {
                    sysState = SystemState.PREPARING_TO_SHOOT;
                }
                break;
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
    public void change_velo(double index){
        veloShooter += index;
    }
    public void change_visage(double index){
        posviseur += index;
    }
    public void Shoot(){
        switch (sysState)
        {
            case WAITING:
                shooter.setVelocity(0);
                break;

            case ASPIRER:
                pont.setPower(-1);
                shooter.setVelocity(shooter_aspirage_puissance);
                viseur.setPosition(1);
                break;
            case PREPARING_TO_SHOOT:
                pont.setPower(1);
                viseur.setPosition(posviseur);
                //setPower_voltage_PIDF();
                shooter.setVelocity(veloShooter);
                break;
            case READY_TO_SHOOT:
                break;

            default:
                shooter.setVelocity(0);
                Dashboard.Telemetry_with_Text("Shooter", "unknown system state used");
                break;
        }
    }
    public void setPower_voltage_PIDF (){
        if (voltage == 0)
            return;
        Pow_shoot = shooter_pidf.Calculate_Power(veloShooter, shooter.getVelocity())/voltage;
        shooter.setPower(Pow_shoot);

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
        shooter_pidf.SetGains(ShooterKP, ShooterKI, ShooterKD, ShooterKF);
        shooter_pidf.SetTolerance(shooter_velo_tolerance);

        pidf = new PIDFCoefficients(ShooterKP_velo, ShooterKI_velo, ShooterKD_velo, voltage != 0 ? ShooterKF_velo * (12/voltage) : ShooterKF_velo);
        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void calculate_postir(double distance_to_goal){
        //veloShooter_auto = (5.79573 * Math.pow(10, -7)) * Math.pow(distance_to_goal, 4) -0.000496167 * Math.pow(distance_to_goal, 3) +0.147211 * Math.pow(distance_to_goal, 2) -15.11259*distance_to_goal +1449.64095;//Definis moi ca
        //posviseur_auto = -0.00168518*distance_to_goal +1.01528;
        //posviseur_auto = -(1.64249*Math.pow(10, -9)) * Math.pow(distance_to_goal, 4)+0.00000128437 * Math.pow(distance_to_goal, 3) -0.000335487 * Math.pow(distance_to_goal, 2) +0.0312942 * distance_to_goal +0.0758803;
        veloShooter_auto = met_la_moi_profond.get(distance_to_goal);
        posviseur_auto = apprend_a_viser.get(distance_to_goal);
    }
    public boolean has_shoot () {
        return sysState == SystemState.READY_TO_SHOOT && !Utils.IsInRange(current_shoot_velo, veloShooter, shooter_velo_tolerance);
    }
    @Override
    public void periodic(){
        update_input();
        updatePID();

        RunStateShooter();
        Shoot();
        TelemetryPacket mon_ptit_truc = new TelemetryPacket();
        mon_ptit_truc.put("velocité du shooter", current_shoot_velo);
        mon_ptit_truc.put("position viseur", current_viseur_pos);
        dashboard.sendTelemetryPacket(mon_ptit_truc);

        telemetry.addData("velocité du shooter", current_shoot_velo);
        telemetry.addData("valeur", Pow_shoot);
        telemetry.addData("voltage du moteur", shooter.getPower());
        telemetry.addData("erreur", shooter_pidf.GetError());
        telemetry.addData("feedforward", shooter_pidf.GetFF());
        telemetry.addData("feedforward * setPoint", shooter_pidf.GetFF()*shooter_pidf.GetSetpoint());
        telemetry.addData("voltage", voltage);
        telemetry.addData("valeur retourné par le pidf", shooter_pidf.calculateInternal(shooter.getVelocity()));
//
        telemetry.update();

    }
}
