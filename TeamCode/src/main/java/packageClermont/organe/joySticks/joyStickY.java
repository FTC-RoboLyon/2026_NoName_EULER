package packageClermont.organe.joySticks;


import org.firstinspires.ftc.robotcore.external.Telemetry;
public class joyStickY {
    private  Telemetry telemetry;
    public joyStickY(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    joySticksFonctions joyStickY = new joySticksFonctions();
    private double nouveauX;
    private double nouveauY;
    private double nVecteur;
    private double angle;
    public double joyStickYPara(float x, double x1, float y, double y1){
        nouveauX = joyStickY.repositionneX(y);
        nouveauY = joyStickY.repositionneY(x);
        nouveauX = joyStickY.smoothX(nouveauX, x1);
        nouveauY = joyStickY.smoothY(nouveauY, y1);
        angle = joyStickY.angle(nouveauX, nouveauY);
        nVecteur = joyStickY.normeVecteur(nouveauX, nouveauY);
        nouveauY = joyStickY.normalisationCirculaireY(nVecteur,nouveauY);
        //nouveauY = joyStickY.deadZoneY(angle, nVecteur, nouveauY);
        nVecteur = joyStickY.Power(nVecteur);
        nouveauY = joyStickY.
        telemetry.addData("angle", angle);
        telemetry.addData("nVecteur", nVecteur);
        telemetry.addData("nouveauX", nouveauX);
        telemetry.addData("nouveauY", nouveauY);
        return nouveauY;
    }
}
