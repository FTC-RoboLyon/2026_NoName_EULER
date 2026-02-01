package packageClermont.organe.joySticks;

public class joyStickY {
    public joyStickY(){

    }
    joySticksFonctions joyStickY = new joySticksFonctions();
    private double nouveauX;
    private double nouveauY;
    private double nVecteur;
    private double angle;
    public double joyStickYPara(float x, double x1, float y, double y1){
        nouveauX = joyStickY.repositionneX(y);
        nouveauY = joyStickY.repositionneY(x);
        nouveauX = joyStickY.smoothX(x, x1);
        nouveauY = joyStickY.smoothY(y, y1);
        nVecteur = joyStickY.normeVecteur(nouveauX, nouveauY);
        angle = joyStickY.angle(nouveauX, nouveauY);
        nouveauY = joyStickY.deadZoneY(angle, nVecteur, nouveauY);
        nouveauY = joyStickY.yPower(nouveauY);
        return nouveauY;
    }
}
