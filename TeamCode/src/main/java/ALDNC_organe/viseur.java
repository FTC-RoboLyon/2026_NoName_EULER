package ALDNC_organe;

import com.qualcomm.robotcore.hardware.Servo;

public class viseur {

    final Servo viseur;

    public viseur (Servo viseur){
        this.viseur = viseur;
        this.viseur.setPosition(0.3);
    }
    double posviseur = 0.6;
    double posviseur_bank = 0.6;
    double posviseur_far = 0.37;

    public void viseur(boolean a, boolean b, boolean y, boolean FG2, boolean FD2) {
        if (a) {
            posviseur = 0.3;
        } else if (b){
            posviseur = posviseur_bank;
        } else if (y) {
            posviseur = posviseur_far;
        } else if (FG2) {
            posviseur += 0.05;
        } else if (FD2){
            posviseur -= 0.05;
        }
        viseur.setPosition(posviseur);
    }
}
