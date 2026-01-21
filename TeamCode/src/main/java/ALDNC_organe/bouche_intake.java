package ALDNC_organe;

import static ALDNC_organe.Constant.INTAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class bouche_intake {

    final DcMotor intake;
    public boolean vIntake = false;

    public bouche_intake (HardwareMap hardware){
        intake = hardware.get(DcMotor.class, INTAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void intake(boolean left_bumper, double Left_Trig) {
        if (left_bumper) {
            intake.setPower(1);
        } else if (Left_Trig > 0.3) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    public int nbeBalles(double distance, int nbeBallesInsideBot){
        if (distance < 25 && !vIntake){
            nbeBallesInsideBot += 1;
            vIntake = true;
        }else if (distance > 25){
            vIntake = false;
        }
        if (nbeBallesInsideBot > 3){
            nbeBallesInsideBot = 3;
        }
        return nbeBallesInsideBot;
    }

    public void intake_simple () {
        intake.setPower(1);
    }

    public void stop_intake () {
        intake.setPower(0);
    }

    public boolean isvIntake () {
        return  vIntake;
    }
}
