package FRC_ALDNC.SubSystem;
import static ALDNC_organe.Constant.LEFT_MOTOR;
import static ALDNC_organe.Constant.RIGHT_MOTOR;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import lib.PidRBL;
@Config
public class Drive_Train extends SubsystemBase {
    DcMotor left_drive, right_drive;
    private double left_motor_power, right_motor_power;
    public static double rotation_P,rotation_I,rotation_D = 0 ;
    public static PidRBL rotattion_Pid = new PidRBL(rotation_P, rotation_I, rotation_D);


    public Drive_Train (HardwareMap hmap){
        left_drive = hmap.get(DcMotor.class, LEFT_MOTOR);
        right_drive = hmap.get(DcMotor.class, RIGHT_MOTOR);

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotattion_Pid.SetTolerance(0.01);
        rotattion_Pid.SetInputLimits(0,Math.PI*2);
        rotattion_Pid.SetContinuous(true);


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
    public void align_rotation (double target_angle, double real_angle){
        left_motor_power = rotattion_Pid.Calculate(target_angle*Math.PI/180, real_angle);
        right_motor_power = -rotattion_Pid.Calculate(target_angle*Math.PI/180, real_angle);
    }
    @Override
    public void periodic(){
        left_drive.setPower(left_motor_power);
        right_drive.setPower(right_motor_power);
    }
}
