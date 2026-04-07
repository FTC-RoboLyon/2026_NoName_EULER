package org.firstinspires.ftc.teamcode.euler.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.euler.RobotTelemetry;
import org.firstinspires.ftc.teamcode.euler.SubSystem;

/**
 * Sous-système gérant le mécanisme de collecte (Intake).
 */
public class Intake implements SubSystem {

    private final DcMotor intakeMotor;
    private IntakeState targetState = IntakeState.IDLE;

    /**
     * Initialise le moteur de l'intake.
     *
     * @param intakeMotor Le moteur physique de l'intake.
     */
    public Intake(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    /**
     * Définit l'intention de collecter des éléments.
     */
    public void collect() {
        targetState = IntakeState.COLLECT;
    }

    /**
     * Définit l'intention d'éjecter des éléments.
     */
    public void eject() {
        targetState = IntakeState.EJECT;
    }

    /**
     * Définit l'intention d'arrêter le mécanisme.
     */
    public void stop() {
        targetState = IntakeState.IDLE;
    }

    /**
     * Alterne entre l'état de collecte et l'arrêt.
     */
    public void toggleCollect() {
        if (targetState == IntakeState.COLLECT) {
            stop();
        } else {
            collect();
        }
    }

    /**
     * Alterne entre l'état d'éjection et l'arrêt.
     */
    public void toggleEject() {
        if (targetState == IntakeState.EJECT) {
            stop();
        } else {
            eject();
        }
    }

    /**
     * Applique la puissance au moteur physique selon l'état cible.
     * Doit être appelée à chaque itération.
     */
    @Override
    public void update() {
        switch (targetState) {
            case COLLECT:
                intakeMotor.setPower(0.7);
                break;
            case EJECT:
                intakeMotor.setPower(-1.0);
                break;
            case IDLE:
            default:
                intakeMotor.setPower(0);
                break;
        }
    }

    @Override
    public RobotTelemetry getTelemetry() {
        return new RobotTelemetry("Intake", "State: " + getState());
    }

    /**
     * Retourne l'intention actuelle du pilote.
     *
     * @return L'état cible demandé (IDLE, COLLECT ou EJECT).
     */
    public IntakeState getTargetState() {
        return targetState;
    }

    /**
     * Retourne l'état réel basé sur la puissance actuelle du moteur.
     * Permet de vérifier si le hardware exécute bien l'ordre.
     *
     * @return L'état physique actuel (IDLE, COLLECT ou EJECT).
     */
    public IntakeState getState() {
        double power = intakeMotor.getPower();
        if (power == 0) {
            return IntakeState.IDLE;
        } else if (power > 0) {
            return IntakeState.COLLECT;
        } else {
            return IntakeState.EJECT;
        }
    }
}
