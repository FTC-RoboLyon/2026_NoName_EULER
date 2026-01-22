package inchallah_organe.inchallah;
import com.qualcomm.robotcore.hardware.Servo;

public class Viseur {
    public Servo viseur;

    public Viseur(Servo viseur){
        this.viseur = viseur;

        viseur.setPosition(posViseur);
    }
    public double posViseurBank = 0.6;
    public double posViseurMid = 0.47;
    public double posViseurFar = 0.42;
    public double posViseur = posViseurBank;

    public void visage(boolean a, boolean b, boolean y, boolean leftFlecheGaucheWpr, boolean leftFlecheDroiteWpr){
        if(b){
            posViseur = posViseurBank;
        }
        if(y){
            posViseur = posViseurMid;
        }
        if(a){
            posViseur = posViseurFar;
        }
        if(leftFlecheGaucheWpr){
            posViseur = posViseur + 0.05;
        }
        if(leftFlecheDroiteWpr){
            posViseur = posViseur - 0.05;
        }
        viseur.setPosition(posViseur);
    }
}
