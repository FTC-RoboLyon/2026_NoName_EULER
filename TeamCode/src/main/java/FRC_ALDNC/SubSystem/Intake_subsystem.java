package FRC_ALDNC.SubSystem;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.COMPTEUR_BALLE;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.INTAKE;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.distance_ou_le_capteur_detecte_balle;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import lib.Dashboard;

public class Intake_subsystem extends SubsystemBase {
    DcMotor intake;
    DistanceSensor compteurBalle;




    public enum WantedState
    {
        STAND_BY,
        COLLECT,
        EJECT
    }

    public enum SystemState
    {
        IDLE,
        INTAKING,
        EJeCT
    }
    public enum capteur_state{
        There_is_nothing,
        There_is_a_ball,
        Has_just_detect_a_ball,
        Has_just_leave_a_ball
    }
    public boolean compteur_flag = false;
    public WantedState m_wantedState = WantedState.STAND_BY;
    public SystemState m_systemState = SystemState.IDLE;
    public static capteur_state capteur_state = Intake_subsystem.capteur_state.There_is_nothing;

    public Intake_subsystem (HardwareMap hmap){
        intake = hmap.get(DcMotor.class, INTAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        compteurBalle = hmap.get(DistanceSensor.class, COMPTEUR_BALLE);
    }

    public void set_wantedState(WantedState m_wantedState) {
        this.m_wantedState = m_wantedState;
    }
    public SystemState getM_systemState() {
        return m_systemState;
    }
    public boolean hasdetected_a_ball (){
        if (compteurBalle.getDistance(DistanceUnit.CM) <= distance_ou_le_capteur_detecte_balle){
            capteur_state = Intake_subsystem.capteur_state.There_is_a_ball;
        }else {
            capteur_state = Intake_subsystem.capteur_state.There_is_nothing;
        }
        if (!compteur_flag){
            if (capteur_state == Intake_subsystem.capteur_state.There_is_a_ball){
                compteur_flag = true;
                return true;
            }
        } else {
            if (capteur_state == Intake_subsystem.capteur_state.There_is_nothing){
                compteur_flag = false;
                return false;
            }
        }

        return false;

    }
    private void RunStateMachine() {
        switch (m_wantedState) {
            case COLLECT:
                if (m_systemState != SystemState.INTAKING)
                    m_systemState = SystemState.INTAKING;
                break;

            case STAND_BY:
                if (m_systemState != SystemState.IDLE)
                    m_systemState = SystemState.IDLE;
            case EJECT:
                if (m_systemState != SystemState.EJeCT)
                    m_systemState = SystemState.EJeCT;
            default:
                //Dashboard.Telemetry_with_Text("Intake", "can't run state machine with an unknown wanted state");
                break;
        }
    }
    @Override
    public void periodic(){
        RunStateMachine();
        switch (m_systemState)
        {
            case IDLE:
                intake.setPower(0.0);
                break;

            case INTAKING:
                intake.setPower(1.0);
                break;
            case EJeCT:
                intake.setPower(-1);

            default:
                //Dashboard.Telemetry_with_Text("Intake", "unknown system state used");
                break;
        }
        Dashboard.Telemetry_with_Text("has detected a ball?", String.valueOf(hasdetected_a_ball()));
    }

}
