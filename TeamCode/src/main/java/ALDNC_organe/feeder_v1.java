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
    private final ElapsedTime timer2 = new ElapsedTime();
    private int b = 0;
    private int c = 0;
    private int d = 0;
    private int e = 0;
    private int f = 0;

    public boolean feederPara(boolean xWasPressed, int nbeBallesInsideBot, boolean isFeeding) {
        if (xWasPressed) {
            b = 1;
        }
        if (b == 1){
            if(nbeBallesInsideBot > e) {
                isFeeding = true;
                if (c == 0) {
                    timer.reset();
                    c = 1;
                }
                if (timer.milliseconds() >= 1000) {
                    isFeeding = false;
                }
                if (!isFeeding){
                    if(d == 0) {
                        timer2.reset();
                        d = 1;
                        f = 0;
                    }
                }
                if (timer2.milliseconds() >= 1000) {
                    if (f == 0) {
                        d = 0;
                        c = 0;
                        e += 1;
                        f = 1;
                    }
                }
            }else if (nbeBallesInsideBot == e){
                b = 0;
                e = 0;
                isFeeding = false;
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
