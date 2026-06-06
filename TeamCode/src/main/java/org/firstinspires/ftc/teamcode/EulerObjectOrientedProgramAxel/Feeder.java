package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
    private final Servo feederServo;
    public static double FeederIdlePos;
    public static double FeederActivePos;
    public Feeder (HardwareMap hmap){
        feederServo = hmap.get(Servo.class, "feeder");

        feederServo.setPosition(FeederIdlePos);
    }
    public void setFeederPos(double feederPos){
        feederServo.setPosition(feederPos);
    }
}
