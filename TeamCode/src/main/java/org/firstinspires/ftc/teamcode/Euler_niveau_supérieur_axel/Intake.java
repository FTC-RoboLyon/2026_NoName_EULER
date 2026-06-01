package org.firstinspires.ftc.teamcode.Euler_niveau_supérieur_axel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;
    public double intake_power;
    public Intake (HardwareMap hmap){
        intake = hmap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void setIntake_power(double Intake_power){
        intake_power = Intake_power;
    }
    public void intake_loop(){
        intake.setPower(intake_power);
    }
}
