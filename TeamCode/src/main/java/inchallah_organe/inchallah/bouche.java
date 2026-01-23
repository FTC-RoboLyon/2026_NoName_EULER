package inchallah_organe.inchallah;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class bouche {
    public DcMotorEx dents;
    public bouche(DcMotorEx dents){
        this.dents = dents;
        dents.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void manger(boolean leftBumper, double leftTrigger ){
        if(leftBumper){
            dents.setPower(1);
        }
        if(leftTrigger > 0.2){
            dents.setPower(-1);
        }
        else{
            dents.setPower(0);
        }
    }
}
