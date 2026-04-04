package org.firstinspires.ftc.teamcode.euler.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Sous-système gérant le mécanisme de tir (Shooter).
 * Utilise {@link DcMotorEx} pour un contrôle précis de la vitesse par PID.
 */
public class Shooter {

    private final DcMotorEx shooterMotor;
    private ShooterState targetState;

    // Vitesse cible en tics par seconde
    private double targetVelocity = 0;
    private static final double VELOCITY_NEAR = 1400;
    private static final double VELOCITY_MIDDLE = 2000;
    private static final double VELOCITY_FAR = 2200;
    private static final double VELOCITY_TOLERANCE = 50;

    /**
     * Initialise le moteur du shooter et configure le mode encodeur.
     *
     * @param shooterMotor Le moteur physique du shooter.
     */
    public Shooter(DcMotor shooterMotor) {
        this.shooterMotor = (DcMotorEx) shooterMotor;
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.targetState = ShooterState.IDLE;
    }

    /**
     * Arrête le moteur du shooter.
     */
    public void stop() {
        targetState = ShooterState.IDLE;
        targetVelocity = 0;
    }

    /**
     * Définit l'intention de tir à courte distance.
     */
    public void shootNear() {
        targetState = ShooterState.SHOOTING;
        targetVelocity = VELOCITY_NEAR;
    }

    /**
     * Définit l'intention de tir à moyenne distance.
     */
    public void shootMiddle() {
        targetState = ShooterState.SHOOTING;
        targetVelocity = VELOCITY_MIDDLE;
    }

    /**
     * Définit l'intention de tir à longue distance.
     */
    public void shootFar() {
        targetState = ShooterState.SHOOTING;
        targetVelocity = VELOCITY_FAR;
    }

    /**
     * Bascule le tir à courte distance (on/off).
     */
    public void toggleShootNear() {
        if (targetState == ShooterState.SHOOTING && targetVelocity == VELOCITY_NEAR) {
            stop();
        } else {
            shootNear();
        }
    }

    /**
     * Bascule le tir à moyenne distance (on/off).
     */
    public void toggleShootMiddle() {
        if (targetState == ShooterState.SHOOTING && targetVelocity == VELOCITY_MIDDLE) {
            stop();
        } else {
            shootMiddle();
        }
    }

    /**
     * Bascule le tir à longue distance (on/off).
     */
    public void toggleShootFar() {
        if (targetState == ShooterState.SHOOTING && targetVelocity == VELOCITY_FAR) {
            stop();
        } else {
            shootFar();
        }
    }

    /**
     * Vérifie si le shooter a atteint la vitesse cible à une tolérance près.
     *
     * @return true si le moteur est stabilisé à la vitesse demandée.
     */
    public boolean isReady() {
        if (targetState == ShooterState.IDLE) return false;
        return Math.abs(shooterMotor.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }

    /**
     * Envoie la commande de vitesse au contrôleur de moteur.
     * Doit être appelée à chaque itération pour assurer la régulation.
     */
    public void update() {
        shooterMotor.setVelocity(targetVelocity);
    }

    /**
     * Retourne l'intention de tir actuelle demandée par le pilote.
     *
     * @return L'état cible (IDLE ou SHOOTING).
     */
    public ShooterState getTargetState() {
        return targetState;
    }

    /**
     * Retourne l'état physique réel basé sur la vitesse actuelle lue par l'encodeur.
     *
     * @return L'état de mouvement réel du moteur (IDLE ou SHOOTING).
     */
    public ShooterState getState() {
        if (Math.abs(shooterMotor.getVelocity()) < 10) {
            return ShooterState.IDLE;
        } else {
            return ShooterState.SHOOTING;
        }
    }

    /**
     * Retourne la vitesse actuelle du moteur lue par l'encodeur.
     *
     * @return Vitesse en tics par seconde.
     */
    public double getActualVelocity() {
        return shooterMotor.getVelocity();
    }
}
