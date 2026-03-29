package org.firstinspires.ftc.teamcode.euler.viseur;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Viseur {
    private final Servo viseurServo;

    // Positions
    public static final double NEAR_POSITION = 0.0;
    public static final double MIDDLE_POSITION = 0.5;
    public static final double FAR_POSITION = 1.0;

    // Temps de trajet estimé
    public static final long TRAVEL_TIME_MS = 300;

    private ViseurInternalState internalState = ViseurInternalState.NEAR;
    private double targetPosition = NEAR_POSITION;
    private double lastCommandedPosition = -1;
    private final ElapsedTime timer = new ElapsedTime();
    private double moveStartTime = 0;

    public Viseur(Servo viseurServo) {
        this.viseurServo = viseurServo;
    }

    public void aimNear() {
        setTarget(ViseurInternalState.NEAR, NEAR_POSITION);
    }

    public void aimMiddle() {
        setTarget(ViseurInternalState.MIDDLE, MIDDLE_POSITION);
    }

    public void aimFar() {
        setTarget(ViseurInternalState.FAR, FAR_POSITION);
    }

    private void setTarget(ViseurInternalState state, double position) {
        if (this.internalState != state) {
            this.internalState = state;
            this.targetPosition = position;
            this.moveStartTime = timer.milliseconds();
        }
    }

    public void update() {
        if (targetPosition != lastCommandedPosition) {
            viseurServo.setPosition(targetPosition);
            lastCommandedPosition = targetPosition;
        }
    }

    public ViseurState getState() {
        // MOVING si on est en train de changer de position (basé sur le timer)
        // Sinon IDLE
        if (timer.milliseconds() - moveStartTime < TRAVEL_TIME_MS) {
            return ViseurState.MOVING;
        }
        return ViseurState.IDLE;
    }
}
