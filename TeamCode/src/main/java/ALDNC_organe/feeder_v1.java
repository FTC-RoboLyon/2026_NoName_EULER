package ALDNC_organe;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class feeder_v1 {

    final Servo feeder;

    public feeder_v1 (Servo feeder){
        this.feeder = feeder;
        this.feeder.setPosition(0.02);
    }

    private final ElapsedTime timer = new ElapsedTime();
    private int b = 0;
    private int e = 0;
    enum Etats { rien, monter, descendre }
    Etats state = Etats.rien;

    public boolean feederPara(boolean xWasPressed, int nbeBallesInsideBot, boolean isFeeding) {
        if (xWasPressed) {
            b = 1;
        }
        if(b == 1){
            switch (state){
                case rien:
                    if (nbeBallesInsideBot > e){
                        state = Etats.monter;
                        timer.reset();
                        break;
                    } else if (nbeBallesInsideBot == e) {
                        b = 0;
                        isFeeding = false;
                        break;
                    }
                    break;

                case monter:
                    isFeeding = true;
                    if (timer.milliseconds() >= 1000){
                        state = Etats.descendre;
                        timer.reset();
                        break;
                    }
                    break;

                case descendre:
                    isFeeding = false;
                    if (timer.milliseconds() >= 1000){
                        state = Etats.rien;
                        e += 1;
                        break;
                    }
                    break;
            }
        }

        return isFeeding;
    }
    public void feeder (boolean isFeeding){
        if (isFeeding) {
            feeder.setPosition(0.2);
        }
        if (!isFeeding) {
            feeder.setPosition(0.02);
        }
    }
    public int compteurBalles(int a, int nbeBallesIn, boolean isFeeding){

        if (a == 0 && isFeeding){
            a = 1;
        }
        if (a == 1 && !isFeeding){
            nbeBallesIn -= 1;
            a = 0;
        }
        if (nbeBallesIn < 0){
            nbeBallesIn = 0;
        }
        return nbeBallesIn;
    }
}
