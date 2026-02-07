package packageClermont.organe.joySticks;

public class joystickX {
    public joystickX(){

    }
    joySticksFonctions joyStickX = new joySticksFonctions();
    private double nouveauX;
    private double nouveauY;
    private double nVecteur;
    private double angle;
    public double joyStickXPara(float x, double x1, float y, double y1){
        nouveauX = joyStickX.repositionneX(y);
        nouveauY = joyStickX.repositionneY(x);
        nouveauX = joyStickX.smoothX(nouveauX, x1);
        nouveauY = joyStickX.smoothY(nouveauY, y1);
        angle = joyStickX.angle(nouveauX, nouveauY);
        nVecteur = joyStickX.normeVecteur(nouveauX, nouveauY);
        nouveauX = joyStickX.normalisationCirculaireX(nVecteur, angle,nouveauX);
        nouveauX = joyStickX.deadZoneX(angle, nVecteur, nouveauX);
        nouveauX = joyStickX.xPower(nouveauX);
        return nouveauX;
    }


}
