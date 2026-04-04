package org.firstinspires.ftc.teamcode.euler.feeder;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Sous-système gérant le mécanisme d'introduction des projectiles (Feeder).
 * Alterne entre une position haute (Push) et une position basse (Idle).
 */
public class Feeder {
    private final Servo feederServo;

    // Positions de configuration
    public static final double PUSH_POSITION = 0.3;
    public static final double IDLE_POSITION = 0.18;

    // Temps estimé pour le mouvement mécanique du servo
    public static final long TRAVEL_TIME_MS = 250;

    private FeederTargetState targetState = FeederTargetState.IDLE;
    private double lastCommandedPosition = -1;
    private final ElapsedTime timer = new ElapsedTime();
    private double moveStartTime = 0;

    /**
     * Initialise le servo du feeder.
     *
     * @param feederServo Le servo physique du feeder.
     */
    public Feeder(Servo feederServo) {
        this.feederServo = feederServo;
    }

    /**
     * Définit l'intention de pousser un projectile vers le shooter.
     */
    public void push() {
        setTarget(FeederTargetState.PUSH);
    }

    /**
     * Définit l'intention de revenir en position basse de repos.
     */
    public void idle() {
        setTarget(FeederTargetState.IDLE);
    }

    /**
     * Alterne entre la position haute et la position basse.
     */
    public void toggle() {
        if (targetState == FeederTargetState.IDLE) {
            push();
        } else {
            idle();
        }
    }

    private void setTarget(FeederTargetState state) {
        if (this.targetState != state) {
            this.targetState = state;
            this.moveStartTime = timer.milliseconds();
        }
    }

    /**
     * Applique la position au servo physique si celle-ci a changé.
     * Doit être appelée à chaque itération.
     */
    public void update() {
        double targetPos = (targetState == FeederTargetState.PUSH) ? PUSH_POSITION : IDLE_POSITION;
        if (targetPos != lastCommandedPosition) {
            feederServo.setPosition(targetPos);
            lastCommandedPosition = targetPos;
        }
    }

    /**
     * Retourne l'intention actuelle de position.
     *
     * @return L'état cible (IDLE ou PUSH).
     */
    public FeederTargetState getTargetState() {
        return targetState;
    }

    /**
     * Estime l'état physique actuel du servo basé sur le temps de trajet théorique.
     *
     * @return L'état physique estimé (IDLE ou MOVING).
     */
    public FeederState getState() {
        if (timer.milliseconds() - moveStartTime < TRAVEL_TIME_MS) {
            return FeederState.MOVING;
        }
        return FeederState.IDLE;
    }
}
