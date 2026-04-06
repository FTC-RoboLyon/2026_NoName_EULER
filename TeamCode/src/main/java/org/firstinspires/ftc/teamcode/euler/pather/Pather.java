package org.firstinspires.ftc.teamcode.euler.pather;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Sous-système gérant le mécanisme Pather utilisant un servo à rotation continue (CRServo).
 * Suit l'architecture de séparation intention/action utilisée dans le projet.
 */
public class Pather {
    private final CRServo patherServo;

    // Vitesses de rotation (Configuration Hardware)
    private static final double BACKWARD_POWER = -1.0;
    private static final double STOP_POWER = 0.0;

    private PatherInternalState targetState = PatherInternalState.IDLE;

    /**
     * Initialise le servo à rotation continue du Pather.
     *
     * @param patherServo Le CRServo physique.
     */
    public Pather(CRServo patherServo) {
        this.patherServo = patherServo;
    }


    /**
     * Définit l'intention de rotation vers l'arrière.
     */
    public void backward() {
        targetState = PatherInternalState.BACKWARD;
    }

    /**
     * Définit l'intention d'arrêter la rotation.
     */
    public void stop() {
        targetState = PatherInternalState.IDLE;
    }

    /**
     * Alterne entre la marche arrière et l'arrêt.
     */
    public void toggleBackward() {
        if (targetState == PatherInternalState.BACKWARD) {
            stop();
        } else {
            backward();
        }
    }

    /**
     * Applique la puissance au servo physique en fonction de l'état cible.
     * Doit être appelée à chaque itération.
     */
    public void update() {
        switch (targetState) {
            case BACKWARD:
                patherServo.setPower(BACKWARD_POWER);
                break;
            case IDLE:
            default:
                patherServo.setPower(STOP_POWER);
                break;
        }
    }

    /**
     * Retourne l'intention actuelle du pilote.
     *
     * @return L'état cible (IDLE, FORWARD ou BACKWARD).
     */
    public PatherInternalState getTargetState() {
        return targetState;
    }

    /**
     * Retourne l'état réel basé sur la puissance actuelle envoyée au servo.
     *
     * @return L'état physique actuel (MOVING ou IDLE).
     */
    public PatherState getState() {
        if (Math.abs(patherServo.getPower()) < 0.05) {
            return PatherState.IDLE;
        } else {
            return PatherState.MOVING;
        }
    }
}
