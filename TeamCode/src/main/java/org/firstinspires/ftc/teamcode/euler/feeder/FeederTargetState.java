package org.firstinspires.ftc.teamcode.euler.feeder;

/**
 * Intentions de position pour le Feeder.
 */
public enum FeederTargetState {
    IDLE,      // Position basse (repos)
    PUSH,      // Position haute (maintien poussé)
    AUTO_FIRE  // Séquence automatique : Pousse puis revient
}
