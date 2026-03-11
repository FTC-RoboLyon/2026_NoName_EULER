package ALDNC_organe;

import static ALDNC_organe.Constant.VISEUR;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

public class viseur {

    public static Servo viseur;


    double posviseur = 0.6;
    double posviseur_bank = 0.6;
    double posviseur_far = 0.42;
    double posviseur_mid = 0.47;

    public viseur (HardwareMap hardware) {
        viseur = hardware.get(Servo.class, VISEUR);
        viseur.setPosition(0.3);
    }

    public void setPosviseur(double Posviseur) {viseur.setPosition(Posviseur);}
    public void setPosBank() {viseur.setPosition(posviseur_bank);}
    public void setPosMid() {viseur.setPosition(posviseur_mid);}
    public void setPosFar() {viseur.setPosition(posviseur_far);}
    public double getPosviseur(){return viseur.getPosition();}

    public void viseur(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a) {
            posviseur = posviseur_far;
        } else if (gamepad1.b){
            posviseur = posviseur_bank;
        } else if (gamepad1.y) {
            posviseur = posviseur_mid;
        } else if (gamepad2.dpadRightWasPressed()) {
            posviseur += 0.05;
        } else if (gamepad2.dpadLeftWasPressed()){
            posviseur -= 0.05;
        }
        viseur.setPosition(posviseur);
    }
}
