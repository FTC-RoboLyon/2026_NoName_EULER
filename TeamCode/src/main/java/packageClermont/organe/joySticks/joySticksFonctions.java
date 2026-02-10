package packageClermont.organe.joySticks;

import static packageClermont.organe.jambes.variablePower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import packageClermont.organe.jambes;



public class joySticksFonctions {
    private double ksmouth = 0.25;
    private double nVecteur;
    private double vDeadZone = 0.05;
    public double PX;
    public double PY;
    public double vecteurTrop;
    public double angle;
    private double nouveauX;
    private double nouveauY;
    private double vPower = 2;
    public joySticksFonctions(){

    }

    public double repositionneX(float y){
        nouveauX = -y;
        return nouveauX;
    }
    public double repositionneY(float x){
        nouveauY = +x;
        return nouveauY;
    }
    public double smoothX(double nouveauX, double vieuxX){
        nouveauX = vieuxX + (nouveauX - vieuxX)*ksmouth;
        return nouveauX;
    }
    public double smoothY(double nouveauY, double vieuxY){
        nouveauY = vieuxY + (nouveauY - vieuxY)*ksmouth;
        return nouveauY;
    }
    public double normeVecteur(double x, double y) {
        nVecteur = Math.sqrt(x * x + y * y);
        if(nVecteur < vDeadZone){
            nVecteur = 0;
        }else if(nVecteur >= vDeadZone){
            nVecteur = (1-vDeadZone)*nVecteur;
        }
        return nVecteur;
    }
    public double normalisationCirculaireX(double nVecteur,double x){
        if(nVecteur > 1){
            x = x/nVecteur;
        }
        return x;
    }
    public double normalisationCirculaireY(double nVecteur,double y){
        if(nVecteur > 1){
           y = y/nVecteur;
        }
        return y;
    }
    public double angle (double x, double y){
        angle = Math.atan(x/y);
        return angle;
    }
    public double deadZoneX(double angle, double nVecteur, double x){
        x = Math.sin(angle)*nVecteur;
        return x;
    }
    public double deadZoneY(double angle, double nVecteur, double y){
        y = Math.cos(angle)*nVecteur;
        return y;
    }
    public double Power(double nVecteur){
        nVecteur = Math.pow(nVecteur, 2);
        return nVecteur;
    }
    public double calculageX(double nVecteur, double angle, double nouveauX){
        nouveauX = nVecteur*Math.sin(angle);
        return nouveauX;
    }
    public double calculageY(double nVecteur, double angle, double nouveauY){
        nouveauY = nVecteur*Math.cos(angle);
        return nouveauY;
    }

}
