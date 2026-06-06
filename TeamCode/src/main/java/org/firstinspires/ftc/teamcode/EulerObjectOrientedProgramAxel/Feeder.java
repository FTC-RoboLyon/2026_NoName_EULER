package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
    private final Servo feederServo;
    public static double FeederIdlePos; //un d de trop je crois, en anglais correct ca donnerait plutot FeederIdlePos
    public static double FeederActivePos; //en anglais correct ca donnerait plutot FeederActivePos
    public Feeder (HardwareMap hmap){
        feederServo = hmap.get(Servo.class, "feeder");

        feederServo.setPosition(FeederIdlePos);
    }
    public void setFeederPos(double feederPos){
        feederServo.setPosition(feederPos);
    } // pas setPosFeeder mais plutôt setFeederPos c'est plus correct


}
