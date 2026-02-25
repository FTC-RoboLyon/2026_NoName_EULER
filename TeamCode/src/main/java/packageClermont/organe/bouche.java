package packageClermont.organe;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class bouche {
    public DcMotorEx dents;
    public bouche(DcMotorEx dents){
        this.dents = dents;
        dents.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void manger(boolean leftBumper, double leftTrigger ){
        if(leftBumper){
            dents.setPower(1);
        } else if(leftTrigger > 0.2){
            dents.setPower(-1);
        }
        else{
            dents.setPower(0);
        }
    }
    public void test(boolean left_bumper){
        if(left_bumper){
            dents.setPower(1);
        }else{
            dents.setPower(0);
        }
    }
}
