package ALDNC_organe;

import static ALDNC_organe.Constant.LEFT_MOTOR;
import static ALDNC_organe.Constant.RIGHT_MOTOR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class jambes {
    final DcMotor left_drive;
    final DcMotor right_drive;
    final double Ticks_Per_Revolution = 0;  // a regler avec la valeur du site
    final double NB_Of_Revolution_Right = 0;
    final double NB_Of_Revolution_Left = 0;


    public jambes (HardwareMap hardware, DcMotor.RunMode runMode) {
        left_drive = hardware.get(DcMotor.class, LEFT_MOTOR);
        right_drive = hardware.get(DcMotor.class, RIGHT_MOTOR);

        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (runMode == DcMotor.RunMode.RUN_USING_ENCODER){
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void drive(double valueLeftMotor, double valueRightMotor) {

        left_drive.setPower(valueLeftMotor);
        right_drive.setPower(valueRightMotor);
    }
    public void run_to_pos(double leftDistance, double rightDistance){
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void turn_left() {
        left_drive.setPower(-1);
        right_drive.setPower(1);
    }
    public void turn_right() {
        left_drive.setPower(1);
        right_drive.setPower(-1);
    }
    public void forward () {
        left_drive.setPower(1);
        right_drive.setPower(1);
    }
    public void backward () {
        left_drive.setPower(-1);
        right_drive.setPower(-1);
    }

    public void stop () {
        left_drive.setPower(0);
        right_drive.setPower(0);
    }

}
