package ALDNC_organe;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class bouche_intake {

    final DcMotor intake;

    public bouche_intake (DcMotor intake){
        this.intake = intake;
        this.intake.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public int nbeBalles(boolean appuyage, int nbeBallesInsideBot, int vIntake){
        if (appuyage && vIntake == 0){
            nbeBallesInsideBot += 1;
            vIntake = 1;
        }else if (!appuyage){
            vIntake = 0;
        }
        if (nbeBallesInsideBot > 3){
            nbeBallesInsideBot = 3;
        }
        return nbeBallesInsideBot;
    }
}
