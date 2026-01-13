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
    int a = 0;
    enum Etats { rien, monter, descendre };
    Etats state = Etats.rien;


    public boolean feederPara(boolean x_was_pressed, boolean isFeeding, boolean isShooting) {
        if(isShooting) {
            if (x_was_pressed) {
                b = 1;
            }
            if (b == 1) {
                switch (state) {
                    case rien:
                        if (e < 3) {
                            state = Etats.monter;
                            timer.reset();
                        } else if (e == 3) {
                            b = 0;
                            isFeeding = false;
                        }
                        break;

                    case monter:
                        isFeeding = true;
                        if (timer.milliseconds() >= 100) {
                            state = Etats.descendre;
                            timer.reset();
                        }
                        break;

                    case descendre:
                        isFeeding = false;
                        if (timer.milliseconds() >= 1000) {
                            state = Etats.rien;
                            e += 1;
                        }
                        break;
                }
            } else if (b == 0) {
                e = 0;

            }
        } else if (!isShooting) {
            b = 0;
            e = 0;
            state = Etats.rien;
        }

        return isFeeding;
    }
    public void feeder (boolean isFeeding, boolean isShooting){
        if(isShooting) {
            if (isFeeding) {
                feeder.setPosition(0.2);
            }
            if (!isFeeding) {
                feeder.setPosition(0);
            }
        } else if (!isShooting) {
            feeder.setPosition(0);
        }
    }
    public int compteurBalles(int nbeBallesIn, boolean isFeeding, boolean isShooting){
        if(isShooting) {
            if (a == 0 && isFeeding) {
                a = 1;
            }
            if (a == 1 && !isFeeding) {
                nbeBallesIn -= 1;
                a = 0;
            }
            if (nbeBallesIn < 0) {
                nbeBallesIn = 0;
            }
        }
        return nbeBallesIn;
    }
}
