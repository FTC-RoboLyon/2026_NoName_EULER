package org.firstinspires.ftc.teamcode.euler.utils;

import java.util.function.BooleanSupplier;

// reponse de gemini à la question
// sachant que c'est un projet ftc, est que la gestion des boutons dans la boucle while est cohérente ?

/**
 * Un wrapper simple pour gérer la détection de pression de bouton (front montant).
 * Permet d'éviter les déclenchements multiples lors d'un appui prolongé.
 */
public class ButtonReader {
    private final BooleanSupplier button;
    private boolean lastState = false;

    public ButtonReader(BooleanSupplier button) {
        this.button = button;
    }

    /**
     * Retourne true uniquement à l'instant où le bouton est pressé.
     *
     * @return true si le bouton vient d'être pressé (transition false -> true)
     */
    public boolean wasJustPressed() {
        boolean currentState = button.getAsBoolean();
        boolean pressed = currentState && !lastState;
        lastState = currentState;
        return pressed;
    }

    /**
     * Retourne l'état actuel du bouton.
     */
    public boolean isDown() {
        lastState = button.getAsBoolean();
        return lastState;
    }
}
