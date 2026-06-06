package org.firstinspires.ftc.teamcode.EulerObjectOrientedProgramAxel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor; //manque le Motor a la fin du nom
    public Intake (HardwareMap hmap){
        intakeMotor = hmap.get(DcMotor.class, "intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void setIntake_power(double Intake_power){
        intakeMotor.setPower(Intake_power);
    }
}
