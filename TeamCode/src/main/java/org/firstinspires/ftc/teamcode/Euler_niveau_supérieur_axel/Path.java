package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Path {
    public CRServo PathServo;
    public double PathPower;
    public Path (HardwareMap hmap){
        PathServo = hmap.get(CRServo.class, "chemin");

        PathServo.setPower(0);
    }
    public void setPathPower(double pathPower){
        PathPower = pathPower;
    }

    public void Path_loop(){
        PathServo.setPower(PathPower);
    }
}
