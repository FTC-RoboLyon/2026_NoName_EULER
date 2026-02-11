package FRC_ALDNC.SubSystem;

import static FRC_ALDNC.Constant.FEEDER;
import static FRC_ALDNC.Constant.posFeed;
import static FRC_ALDNC.Constant.posFeedrepos;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import lib.Dashboard;
import packageClermont.organe.Feeder;

public class Feeder_subsystem extends SubsystemBase {
    Servo feeder;
    public  enum FeederState{
        Haut, bas
    }
    public  enum Feeder_wanted_state{
        Haut,bas
    }
    private Feeder_wanted_state to_feed_or_not_to_feed = Feeder_wanted_state.bas;
    private FeederState feed_state = FeederState.bas;

    public Feeder_subsystem(HardwareMap hmap){
        feeder = hmap.get(Servo.class, FEEDER);
        feeder.setPosition(posFeedrepos);
    }
    public void setfeeder_wanted_state(Feeder_wanted_state feederWantedState){to_feed_or_not_to_feed = feederWantedState;}
    public FeederState getFeed_state(){return feed_state;}

    public void RunStateFeeder(){
        switch (to_feed_or_not_to_feed){
            case Haut:
                if (feed_state != FeederState.Haut){
                    feed_state = FeederState.Haut;
                }
                break;
            case bas:
                if (feed_state != FeederState.bas)
                    feed_state = FeederState.bas;
                break;
            default:
                //Dashboard.Telemetry_with_Text("Feeder", "unknown system state used");
                break;
        }switch (feed_state){
            case bas:
                break;
            case Haut:
                if (feeder.getPosition() == posFeed)
                    feed_state = FeederState.bas;
                break;
        }
    }
    @Override
    public  void  periodic(){
        RunStateFeeder();
        switch (feed_state){
            case Haut:
                feeder.setPosition(posFeed);
                break;
            case bas:
                feeder.setPosition(posFeedrepos);
                break;
        }
    }
}
