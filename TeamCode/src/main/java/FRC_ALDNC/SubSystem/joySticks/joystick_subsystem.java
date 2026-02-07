package FRC_ALDNC.SubSystem.joySticks;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

public class joystick_subsystem extends SubsystemBase {
    private double kp = 0.95;
    private double vDeadZone = 0.025;
    private double vPower = 2;


    private double X_brut, Y_brut;
    private double ancien_X, ancien_Y;

    private double nVecteur;
    public double angle;
    private double nouveauX;
    private double nouveauY;
    enum  Witch_stick{
        left, right
    }
    private Witch_stick stick;
    private Gamepad gamepad;

    public joystick_subsystem(double kp, double vDeadZone, double vPower, Gamepad gamepad, String stick_pos){
        this.kp = kp;
        this.vDeadZone = vDeadZone;
        this.vPower = vPower;
        this.gamepad = gamepad;
        if (Objects.equals(stick_pos, "left")){
            stick = Witch_stick.left;
        } else if (Objects.equals(stick_pos, "right")) {
            stick = Witch_stick.right;
        }

    }
    public void actualise(){
        if (stick == Witch_stick.left){
            X_brut = gamepad.left_stick_x;
            Y_brut = gamepad.left_stick_y;
        } else if (stick == Witch_stick.right){
            X_brut = gamepad.right_stick_x;
            Y_brut = gamepad.right_stick_y;
        }
        nouveauX = X_brut;
        nouveauY = Y_brut;
    }
    public void repositionneX(){
        nouveauX = -Y_brut;
    }
    public void repositionneY(){
        nouveauY = X_brut;
    }
    public void smoothX(double vieuxX, double NouveauX){
        NouveauX = vieuxX + (nouveauX - vieuxX)*kp;
    }
    public void smoothY(double vieuxY, double NouveauY){
        NouveauY = vieuxY + (nouveauY - vieuxY)*kp;
    }
    public void calculateNormeVecteur() {
        nVecteur = Math.sqrt(nouveauX*nouveauX+ nouveauY*nouveauY);
        if(nVecteur > 1){
            nVecteur  = 1;
            nouveauY = Math.sin(angle);
            nouveauX = Math.cos(angle);
        }
    }
    public void calculateAngle(){
        angle = Math.atan(nouveauY/nouveauX);
    }
    public double deadZoneX(double angle, double nVecteur, double x){
        x = Math.sin(angle)*nVecteur;
        return x;
    }
    public double deadZoneY(double angle, double nVecteur, double y){
        y = Math.cos(angle)*nVecteur;
        return y;
    }
    public void xPower(){
        nouveauX = Math.pow(nouveauX, vPower);

    }
    public void yPower(){
        nouveauY = Math.pow(nouveauY, vPower);

    }
    @Override
    public  void periodic(){
        ancien_X = nouveauX;
        ancien_Y = nouveauY;

        actualise();
        calculateAngle();
        calculateNormeVecteur();
        smoothX(ancien_X, nouveauX);
        smoothY(ancien_Y, nouveauY);
        xPower();
        yPower();


    }

}
