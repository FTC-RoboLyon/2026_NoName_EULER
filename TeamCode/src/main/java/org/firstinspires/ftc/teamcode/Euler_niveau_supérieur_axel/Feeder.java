package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
    private Servo feederServo;
    public static double PosFeederIddle; //un d de trop je crois, en anglais correct ca donnerait plutot FeederIdlePos
    public static double PosFeederActive; //en anglais correct ca donnerait plutot FeederActivePos
    public double actualPosFeeder; //en anglais correct ca donnerait plutot FeederActualPos
    public Feeder (HardwareMap hmap){
        feederServo = hmap.get(Servo.class, "feeder");

        feederServo.setPosition(PosFeederIddle);
    }
    public void setPosFeeder(double feederPos){
        actualPosFeeder = feederPos;
    } // pas setPosFeeder mais plutôt setFeederPos c'est plus correct

    public void feeder_loop(){
        feederServo.setPosition(actualPosFeeder);
    }
}
