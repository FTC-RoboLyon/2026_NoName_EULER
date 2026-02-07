package packageClermont.organe.joySticks;

public class joySticksFonctions {
    private double kp = 0.8;
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
    public double smoothX(double vieuxX, double nouveauX){
        nouveauX = vieuxX + (nouveauX - vieuxX)*kp;
        return nouveauX;
    }
    public double smoothY(double vieuxY, double nouveauY){
        nouveauY = vieuxY + (nouveauY - vieuxY)*kp;
        return nouveauY;
    }
    public double normeVecteur(double x, double y) {
        nVecteur = Math.sqrt(x * x + y * y);
        if(nVecteur < vDeadZone){
            nVecteur = 0;
        }else if(nVecteur >= vDeadZone){
            nVecteur = 1-vDeadZone*(nVecteur-vDeadZone);
        }
        return nVecteur;
    }
    public double normalisationCirculaireX(double nVecteur, double angle, double x){
        if(nVecteur > 1){
            vecteurTrop = nVecteur -1;
            PX = Math.sin(angle)*vecteurTrop;
            x = x-PX;
        }
        return x;
    }
    public double normalisationCirculaireY(double nVecteur, double angle, double y){
        if(nVecteur > 1){
            vecteurTrop = nVecteur - 1;
            PY = Math.cos(angle)*vecteurTrop;
            y = y-PY;
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
    public double xPower(double x){
        x = Math.pow(x, vPower);
        return x;
    }
    public double yPower(double y){
        y = Math.pow(y, vPower);
        return y;
    }

}
