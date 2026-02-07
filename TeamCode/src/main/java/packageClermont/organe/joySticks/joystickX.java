package packageClermont.organe.joySticks;


import org.firstinspires.ftc.robotcore.external.Telemetry;
public class joystickX {
    private Telemetry telemetry;
    public joystickX(Telemetry telemetry){
        this.telemetry = telemetry;
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
        nouveauX = joyStickX.normalisationCirculaireX(nVecteur,nouveauX);
        //nouveauX = joyStickX.deadZoneX(angle, nVecteur, nouveauX);
        nVecteur = joyStickX.Power(nVecteur);
        //nouveauX = joyStickX.calculageX(nVecteur, angle, nouveauX);
        //telemetry.addData("angle", angle);
        //telemetry.addData("nVecteur", nVecteur);
        telemetry.addData("nouveauX", nouveauX);
        telemetry.addData("nouveauY", nouveauY);
        return nouveauX;
    }


}
