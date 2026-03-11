package packageClermont.organe;

import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
    public Servo feeder;
    public Feeder(Servo feeder){
        this.feeder = feeder;
        feeder.setPosition(posfeederBas);
    }
    public double posfeederHaut = 0.3;
    public double posfeederBas = 0.125;
    public void grosseCommition(boolean xpr, boolean xrl){
        if(xpr) {
            feeder.setPosition(posfeederHaut);
        }
        if(xrl){
            feeder.setPosition(posfeederBas);
        }
    }
}
