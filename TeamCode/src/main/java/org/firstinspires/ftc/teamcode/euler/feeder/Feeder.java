package org.firstinspires.ftc.teamcode.euler.feeder;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Sous-système gérant le mécanisme d'introduction des projectiles (Feeder).
 * Permet un contrôle manuel (Haut/Bas) ou une séquence automatique de tir.
 */
public class Feeder {
    private final Servo feederServo;

    // Positions de configuration
    public static final double PUSH_POSITION = 0.3;
    public static final double IDLE_POSITION = 0.18;

    // Temps de trajet du servo (ms)
    public static final long TRAVEL_TIME_MS = 250;
    // Temps de maintien en position haute lors d'un autoFire (ms)
    public static final long HOLD_TIME_MS = 150;

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
     * Définit l'intention de pousser manuellement.
     */
    public void push() {
        setTarget(FeederTargetState.PUSH);
    }

    /**
     * Définit l'intention de revenir au repos manuellement.
     */
    public void idle() {
        setTarget(FeederTargetState.IDLE);
    }

    /**
     * Déclenche une séquence automatique : Pousse, attend, puis revient.
     */
    public void autoFire() {
        setTarget(FeederTargetState.AUTO_FIRE);
    }

    /**
     * Alterne entre la position haute et basse (Contrôle manuel).
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
     * Applique la position et gère la séquence automatique si nécessaire.
     * Doit être appelée à chaque itération.
     */
    public void update() {
        double currentPos = targetState == FeederTargetState.IDLE ? IDLE_POSITION : PUSH_POSITION;

        // Gestion de la séquence automatique (FSM interne)
        if (targetState == FeederTargetState.AUTO_FIRE) {
            long elapsed = (long) (timer.milliseconds() - moveStartTime);
            // Si on a fini de pousser et de maintenir (Trajet + Maintien)
            if (elapsed > (TRAVEL_TIME_MS + HOLD_TIME_MS)) {
                idle(); // Revient automatiquement au repos
            }
        }

        // Application hardware
        if (currentPos != lastCommandedPosition) {
            feederServo.setPosition(currentPos);
            lastCommandedPosition = currentPos;
        }
    }

    /**
     * Retourne l'intention actuelle.
     */
    public FeederTargetState getTargetState() {
        return targetState;
    }

    /**
     * Estime l'état physique actuel.
     */
    public FeederState getState() {
        if (timer.milliseconds() - moveStartTime < TRAVEL_TIME_MS) {
            return FeederState.MOVING;
        }
        return FeederState.IDLE;
    }
}
