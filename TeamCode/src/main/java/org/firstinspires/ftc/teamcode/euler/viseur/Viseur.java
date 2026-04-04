package org.firstinspires.ftc.teamcode.euler.viseur;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Viseur - Contrôle la position du servo d'inclinaison pour le tir.
 * Architecture robuste basée sur des états internes et une mise à jour différée.
 */
public class Viseur {
    private final Servo viseurServo;

    // Positions (Configuration Hardware)
    private static final double NEAR_POSITION = 0.0;
    private static final double MIDDLE_POSITION = 0.5;
    private static final double FAR_POSITION = 1.0;

    // Paramètres temporels
    public static final long TRAVEL_TIME_MS = 300;

    private ViseurTargetState targetState = ViseurTargetState.FAR;
    private double lastCommandedPosition = -1;
    private final ElapsedTime timer = new ElapsedTime();
    private double moveStartTime = 0;

    public Viseur(Servo viseurServo) {
        this.viseurServo = viseurServo;
    }

    public void aimNear() {
        setTarget(ViseurTargetState.NEAR);
    }

    public void aimMiddle() {
        setTarget(ViseurTargetState.MIDDLE);
    }

    public void aimFar() {
        setTarget(ViseurTargetState.FAR);
    }

    private void setTarget(ViseurTargetState state) {
        if (this.targetState != state) {
            this.targetState = state;
            this.moveStartTime = timer.milliseconds();
        }
    }

    /**
     * Traduit l'état interne en position réelle pour le servo.
     */
    public void update() {
        double targetPos;
        switch (targetState) {
            case MIDDLE:
                targetPos = MIDDLE_POSITION;
                break;
            case FAR:
                targetPos = FAR_POSITION;
                break;
            case NEAR:
            default:
                targetPos = NEAR_POSITION;
                break;
        }

        if (targetPos != lastCommandedPosition) {
            viseurServo.setPosition(targetPos);
            lastCommandedPosition = targetPos;
        }
    }

    /**
     * Retourne l'intention actuelle.
     */
    public ViseurTargetState getTargetState() {
        return targetState;
    }

    /**
     * Retourne l'état physique estimé (MOVING ou IDLE).
     */
    public ViseurState getState() {
        if (timer.milliseconds() - moveStartTime < TRAVEL_TIME_MS) {
            return ViseurState.MOVING;
        }
        return ViseurState.IDLE;
    }
}
