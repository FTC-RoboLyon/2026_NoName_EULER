package FRC_ALDNC.SubSystem;

import static FRC_ALDNC.CONSTANT.constante_feeder.FEEDER;
import static FRC_ALDNC.CONSTANT.constante_feeder.posFeed;
import static FRC_ALDNC.CONSTANT.constante_feeder.posFeedrepos;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FRC_ALDNC.ALDNC_container;

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
    private ALDNC_container robot;

    public Feeder_subsystem(HardwareMap hmap, ALDNC_container RObot){
        robot = robot;
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
        }
    }
    public double get_Pos (){return feeder.getPosition();}
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
