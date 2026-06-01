package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
    private Servo feederServo;
    public static double PosFeederIddle;
    public static double PosFeederActive;
    public double actualPosFeeder;
    public Feeder (HardwareMap hmap){
        feederServo = hmap.get(Servo.class, "feeder");

        feederServo.setPosition(PosFeederIddle);
    }
    public void setPosFeeder(double feederPos){
        actualPosFeeder = feederPos;
    }

    public void feeder_loop(){
        feederServo.setPosition(actualPosFeeder);
    }
}
