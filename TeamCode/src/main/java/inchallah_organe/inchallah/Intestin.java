package inchallah_organe.inchallah;

import com.qualcomm.robotcore.hardware.Servo;

public class Intestin {
    public Servo rectum;
    public Intestin(Servo rectum){
        this.rectum = rectum;
        rectum.setPosition(posfeederBas);
    }
    public double posfeederHaut = 0.3;
    public double posfeederBas = 0.125;
    public void grosseCommition(boolean xpr, boolean xrl){
        if(xpr) {
            rectum.setPosition(posfeederHaut);
        }
        if(xrl){
            rectum.setPosition(posfeederBas);
        }
    }
}
