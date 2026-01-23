package ALDNC_organe;

import static ALDNC_organe.Constant.FEEDER;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class feeder_v1 {

    public static Servo feeder;
    public static double PosFeed = 0.4;
    public static double PosRepos = 0.125;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean b = false;
    private int e = 0;
    int a = 0;
    int vIsShooting = 0;
    enum Etats { rien, monter, descendre };
    Etats state = Etats.rien;


    public feeder_v1 (HardwareMap hardware){
        feeder = hardware.get(Servo.class, FEEDER);
        feeder.setPosition(0.10);
    }

    public double getposition(){return feeder.getPosition();}
    public void setPosFeed(){feeder.setPosition(PosFeed);}
    public void setPosRepos(){feeder.setPosition(PosRepos);}
    public boolean isPosFeed(){return Math.abs(feeder.getPosition()-PosFeed) <=0.01;}

    public void Feeder(Gamepad gamepad1){
        if (gamepad1.xWasPressed()){
        feeder.setPosition(PosFeed);
    } else if (gamepad1.xWasReleased()) {
            feeder.setPosition(PosRepos);
        }
    }


    public boolean feederPara(boolean x_was_pressed, boolean isFeeding, boolean isShooting) {
        if (x_was_pressed) {
            b = !b;
        }
        if (isShooting) {
            vIsShooting = 0;
            if (b) {
                switch (state) {
                    case rien:
                        if (e < 3) {
                            state = Etats.monter;
                            timer.reset();
                        } else if (e == 3) {
                            b = false;
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
                if (!b) {
                    e = 0;
                    state = Etats.rien;

                }
            } else if (!isShooting && vIsShooting == 0) {
                vIsShooting = 1;
                b = false;
                e = 0;
                state = Etats.rien;
            }

            if (!isShooting) {
                b = false;
                e = 0;
                state = Etats.rien;
            }


        }
        return isFeeding;
    }
    public void feed(boolean isfeeding, boolean isshooting){
        if(isshooting) {
            if (isfeeding) {
                feeder.setPosition(0.2);
            }
            if (!isfeeding) {
                feeder.setPosition(0);
            }
        } else {
            feeder.setPosition(0);
        }
    }
    public int compteurBalles(int nbeBallesIn, boolean isFEeding, boolean isSHooting){
        if(isSHooting) {
            if (a == 0 && isFEeding) {
                a = 1;
            }
            if (a == 1 && !isFEeding) {
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
