package packageClermont.organe;
import com.qualcomm.robotcore.hardware.Servo;

public class Viseur {
    public Servo viseur;

    public Viseur(Servo viseur){
        this.viseur = viseur;

        viseur.setPosition(posViseur);
    }
    public double posViseurBank = 0.8;
    public double posViseurMid = 0.47;
    public double posViseurFar = 0.42;
    public double posViseur = posViseurBank;

    public void visage(boolean a, boolean b, boolean y, boolean leftFlecheGaucheWpr, boolean leftFlecheDroiteWpr){
        if(b){
            posViseur = posViseurBank;
        } else if(y){
            posViseur = posViseurMid;
        } else if(a){
            posViseur = posViseurFar;
        }
        if(leftFlecheGaucheWpr){
            posViseur = posViseur + 0.05;
        } else if(leftFlecheDroiteWpr){
            posViseur = posViseur - 0.05;
        }
        viseur.setPosition(posViseur);
    }
    public boolean test1(boolean a, boolean b, boolean y, boolean leftFlecheGaucheWpr, boolean leftFlecheDroiteWpr, boolean caca){
        if(a||b||y||leftFlecheGaucheWpr||leftFlecheDroiteWpr){
            caca = true;
        }
        return caca;
    }
}
