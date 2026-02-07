package FRC_ALDNC.SubSystem;
import static ALDNC_organe.Constant.LEFT_MOTOR;
import static ALDNC_organe.Constant.RIGHT_MOTOR;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive_Train extends SubsystemBase {
    DcMotor left_drive, right_drive;
    private double left_motor_power, right_motor_power;

    public Drive_Train (HardwareMap hmap){
        left_drive = hmap.get(DcMotor.class, LEFT_MOTOR);
        right_drive = hmap.get(DcMotor.class, RIGHT_MOTOR);

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void turn_left() {
        left_motor_power = -1;
        right_motor_power = 1;
    }
    public void turn_right() {
        left_motor_power = 1;
        right_motor_power = -1;
    }
    public void forward() {
        left_motor_power = 1;
        right_motor_power = 1;
    }
    public void backward () {
        left_motor_power = -1;
        right_motor_power = -1;
    }

    public void stop () {
        left_drive.setPower(0);
        right_drive.setPower(0);
    }
    public void drive(double valueLeftMotor, double valueRightMotor) {

        left_motor_power = valueLeftMotor;
        right_motor_power = valueRightMotor;
    }
    @Override
    public void periodic(){
        left_drive.setPower(left_motor_power);
        right_drive.setPower(right_motor_power);
    }
}
