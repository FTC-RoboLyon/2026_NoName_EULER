package ALDNC_organe;

import com.qualcomm.robotcore.hardware.Servo;

public class feeder_v1 {

    final Servo feeder;

    public feeder_v1 (Servo feeder){
        this.feeder = feeder;
        this.feeder.setPosition(0.02);
    }

    public void feeder(boolean xpr, boolean xrl) {
        if (xpr) {
            feeder.setPosition(0.2);

        } else if (xrl) {
            feeder.setPosition(0.02);
        }
    }

}
