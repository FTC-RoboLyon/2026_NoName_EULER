package ALDNC_organe;

import com.qualcomm.robotcore.hardware.Servo;

public class viseur {

    final Servo viseur;

    public viseur (Servo viseur){
        this.viseur = viseur;
        this.viseur.setPosition(0.3);
    }

    public void viseur(boolean a, boolean b, boolean y, boolean FG2, boolean FD2, double posviseur, double posTirfar, double posTirbank) {
        if (a) {
            posviseur = 0.3;
        } else if (b){
            posviseur = posTirbank;
        } else if (y) {
            posviseur = posTirfar;
        } else if (FG2) {
            posviseur += 0.05;
        } else if (FD2){
            posviseur -= 0.05;
        }
        viseur.setPosition(posviseur);
    }
}
