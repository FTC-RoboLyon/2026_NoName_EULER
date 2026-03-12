package FRC_ALDNC.SubSystem;

import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.coef_smooth_stickleft;
import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.coef_smooth_stickright;
import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.deadzone_stickright;
import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.deadzone_sticktleft;
import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.vpower_stickleft;
import static FRC_ALDNC.CONSTANT.constante_joystick_and_base.vpower_stickright;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class joystick_subsystem extends SubsystemBase {


    public double coeff_smooth, vdeadzone, vpower;

    private double nVecteur;
    public double angle;

    private double ancien_X, ancien_Y;
    private double nouveauX, nouveauY;
    private double finalX, finalY;

    public enum  Witch_stick{
        left, right
    }
    private Witch_stick stick;
    private GamepadEx gamepad;


    public joystick_subsystem(GamepadEx gamepad, Witch_stick witchStick){

        this.gamepad = gamepad;
        if (witchStick == Witch_stick.left){
            stick = Witch_stick.left;
            coeff_smooth = coef_smooth_stickleft;
            vdeadzone = deadzone_sticktleft;
            vpower = vpower_stickleft;
        } else if (witchStick == Witch_stick.right) {
            stick = Witch_stick.right;
            coeff_smooth = coef_smooth_stickright;
            vdeadzone = deadzone_stickright;
            vpower = vpower_stickright;
        }

    }
    public void actualise(){
        if (stick == Witch_stick.left){
            nouveauX = gamepad.getLeftY();
            nouveauY = gamepad.getLeftX();
        } else if (stick == Witch_stick.right){
            nouveauX = -gamepad.getRightY();
            nouveauY = gamepad.getRightX();
        }
    }
    public void smoothX(double vieuxX){
        nouveauX = vieuxX + (nouveauX - vieuxX)*coeff_smooth;
    }
    public void smoothY(double vieuxY){
        nouveauY = vieuxY + (nouveauY - vieuxY)*coeff_smooth;
    }
    public void calculateNormeVecteur() {
        nVecteur = Math.sqrt(nouveauX*nouveauX+ nouveauY*nouveauY);
        if(nVecteur < vdeadzone){
            nVecteur = 0;
        }else if(nVecteur >= vdeadzone){
            nVecteur = (1-vdeadzone)*nVecteur;
        }
    }
    public void normalisationCirculaireX(){
        if(nVecteur > 1){
            nouveauX = nouveauX/nVecteur;
        }
    }
    public void normalisationCirculaireY(){
        if(nVecteur > 1){
            nouveauY= nouveauY/nVecteur;
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
        nouveauX = nouveauX >= 0 ? Math.pow(nouveauX, vpower): -Math.pow(nouveauX, vpower);
    }
    public void yPower(){
        nouveauY = nouveauY >= 0 ? Math.pow(nouveauY, vpower): -Math.pow(nouveauY, vpower);

    }

    public double getY (){
        return finalY;
    }
    public double getX (){
        return finalX;
    }

    @Override
    public void periodic(){
        ancien_X = nouveauX;
        ancien_Y = nouveauY;
        actualise();
        calculateAngle();
        calculateNormeVecteur();
        smoothX(ancien_X);
        smoothY(ancien_Y);
        //xPower();
        //yPower();
        finalX = nouveauX;
        finalY = nouveauY;
    }

}
