package ALDNC_organe;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class jambes {
    final DcMotor left_motor;
    final DcMotor right_motor;

    public jambes (DcMotor left_motor, DcMotor right_motor) {
        this.left_motor = left_motor;
        this.right_motor = right_motor;

        this.left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.right_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(float valueLeftMotor, float valueRightMotor, boolean a) {
        if (a) {
            valueLeftMotor /= 2;
            valueRightMotor /= 2;
        }
        left_motor.setPower(valueLeftMotor);
        right_motor.setPower(valueRightMotor);
    }

}
