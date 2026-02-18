package FRC_ALDNC.SubSystem;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ENCODERD;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ENCODEURG;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.LEFT_MOTOR;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.RIGHT_MOTOR;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.rotation_D;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.rotation_I;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.rotation_P;

import android.os.FileUriExposedException;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Base64;

import lib.PidRBL;
public class Drive_Train extends SubsystemBase {
    DcMotorEx left_drive, right_drive, encoderD, encoderG;
    private double left_motor_power, right_motor_power, x, y, valueEncoderD, valueEncoderG, vielleValueD, vielleValueG, angle = 0, rayon = 2.54, CPR = 8192, DG, DD, DL = 1.56;
    public static PidRBL rotattion_Pid = new PidRBL(rotation_P, rotation_I, rotation_D);


    public Drive_Train (HardwareMap hmap){
        left_drive = hmap.get(DcMotorEx.class, LEFT_MOTOR);
        right_drive = hmap.get(DcMotorEx.class, RIGHT_MOTOR);

        encoderG = hmap.get(DcMotorEx.class, ENCODEURG);
        encoderD = hmap.get(DcMotorEx.class, ENCODERD);

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderG.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public void drive(double forward, double turn) {
        left_motor_power = forward + turn;
        right_motor_power = forward - turn;
    }
    public void align_rotation (double target_angle, double real_angle, Telemetry telemetry){
        double turn = rotattion_Pid.Calculate(target_angle*Math.PI/180, real_angle);
        telemetry.addData("left motor power", turn);
        telemetry.addData("reight motor power", -turn);
        telemetry.addLine(rotattion_Pid.GetState());
    }
    public void slow_down (double ralentisseur){
        left_motor_power /= ralentisseur;
        right_motor_power /= ralentisseur;
    }
    private void calculateValuesEncoderDetG(){
        valueEncoderD = encoderD.getCurrentPosition() - vielleValueD;
        valueEncoderG = encoderG.getCurrentPosition() - vielleValueG;
        vielleValueD = encoderD.getCurrentPosition();
        vielleValueG = encoderG.getCurrentPosition();
    }
    private void calculateDistanceGetD(){
        DG = Math.PI*2*rayon / CPR;
        DG = DG*valueEncoderG;
        DD = Math.PI*2*rayon / CPR;
        DD = DD*valueEncoderD;
    }
    private void calculateAngleRadiant(){
        angle = (DD - DG)*DL;
    }
    private void calculateX(){
        x = 
    }
    @Override
    public void periodic(){
        left_drive.setPower(left_motor_power);
        right_drive.setPower(right_motor_power);
    }
}
