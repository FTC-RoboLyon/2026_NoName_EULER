package ALDNC_organe;

import static ALDNC_organe.Constant.LEFT_MOTOR;
import static ALDNC_organe.Constant.RIGHT_MOTOR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class jambes {
    final DcMotor left_motor;
    final DcMotor right_motor;



    public jambes (HardwareMap hardware) {
        left_motor = hardware.get(DcMotor.class, LEFT_MOTOR);
        right_motor = hardware.get(DcMotor.class, RIGHT_MOTOR);

        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double valueLeftMotor, double valueRightMotor) {

        left_motor.setPower(valueLeftMotor);
        right_motor.setPower(valueRightMotor);
    }
    public void run_to_pos(double leftDistance, double rightDistance){
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void turn_left() {
        left_motor.setPower(-1);
        right_motor.setPower(1);
    }
    public void turn_right() {
        left_motor.setPower(1);
        right_motor.setPower(-1);
    }
    public void forward () {
        left_motor.setPower(1);
        right_motor.setPower(1);
    }
    public void backward () {
        left_motor.setPower(-1);
        right_motor.setPower(-1);
    }

    public void stop () {
        left_motor.setPower(0);
        right_motor.setPower(0);
    }

}
