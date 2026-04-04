package org.firstinspires.ftc.teamcode.euler.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Sous-système gérant le mécanisme de tir (Shooter).
 * Utilise {@link DcMotorEx} pour un contrôle précis de la vitesse par PID.
 * Adapte la vélocité cible en fonction de la tension de la batterie.
 */
public class Shooter {

    private final DcMotorEx shooterMotor;
    private final VoltageSensor voltageSensor;
    private ShooterState targetState;

    // Vitesse cible nominale (pour 12V) en tics par seconde
    private double targetVelocity = 0;
    private static final double VELOCITY_NEAR = 1400;
    private static final double VELOCITY_MIDDLE = 2100;
    private static final double VELOCITY_FAR = 2400;
    private static final double VELOCITY_TOLERANCE = 50;

    /**
     * Initialise le moteur du shooter et le capteur de tension.
     *
     * @param shooterMotor  Le moteur physique du shooter.
     * @param voltageSensor Le capteur de tension de la batterie.
     */
    public Shooter(DcMotor shooterMotor, VoltageSensor voltageSensor) {
        this.shooterMotor = (DcMotorEx) shooterMotor;
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.voltageSensor = voltageSensor;
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
     * Vérifie si le shooter a atteint la vitesse cible adaptée à la tension.
     *
     * @return true si le moteur est stabilisé à la vitesse compensée.
     */
    public boolean isReady() {
        if (targetState == ShooterState.IDLE) return false;
        return Math.abs(shooterMotor.getVelocity() - getCompensatedVelocity()) < VELOCITY_TOLERANCE;
    }

    /**
     * Calcule la vélocité compensée en fonction de la tension batterie.
     * Formule : V_cible = V_nominale * (12.0 / V_actuelle)
     * Cela permet de garder une puissance de tir constante même quand la batterie faiblit.
     */
    private double getCompensatedVelocity() {
        double voltage = voltageSensor.getVoltage();
        if (voltage < 1.0) {
            voltage = 12.0; // Sécurité si lecture erronée
        }
        return targetVelocity * (12.0 / voltage);
    }

    /**
     * Envoie la commande de vitesse compensée au contrôleur de moteur.
     * Doit être appelée à chaque itération.
     */
    public void update() {
        shooterMotor.setVelocity(getCompensatedVelocity());
    }

    /**
     * Retourne l'intention du pilote.
     */
    public ShooterState getTargetState() {
        return targetState;
    }

    /**
     * Retourne l'état physique réel basé sur la vitesse actuelle.
     */
    public ShooterState getState() {
        if (Math.abs(shooterMotor.getVelocity()) < 10) {
            return ShooterState.IDLE;
        } else {
            return ShooterState.SHOOTING;
        }
    }

    /**
     * Retourne la vitesse actuelle lue par l'encodeur.
     */
    public double getActualVelocity() {
        return shooterMotor.getVelocity();
    }
}
