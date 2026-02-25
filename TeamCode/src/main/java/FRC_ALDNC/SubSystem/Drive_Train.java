package FRC_ALDNC.SubSystem;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ENCODERD;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ENCODEURG;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.LEFT_MOTOR;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.RIGHT_MOTOR;

import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.ff_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.p_rotation;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.rotation_D;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.rotation_I;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.rotation_P;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.seuilDriveShooter;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.tolerance_go_angle;
import static FRC_ALDNC.CONSTAAANT_CESTMOILEBON.tolerence_rotation;

import android.os.FileUriExposedException;

//import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;



import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Base64;

import FRC_ALDNC.ALDNC_container;
import lib.PidRBL;
import lib.Utils;

public class Drive_Train extends SubsystemBase {
    DcMotorEx left_drive, right_drive/*, encoderD, encoderG*/;
    private double left_motor_power, right_motor_power, x, y, valueEncoderD, valueEncoderG, vielleValueD, vielleValueG, vieuxX, vieuxY, vieuxAngle, angle, angleDegrees, rayon = 5.7, CPR = 580, DG, DD, DL = 36, h;
    // truc de pid pour l'odometrie
    private double p = 0.05, target, erreur, targetDegrees, xOdo, yOdo;
    private double targetD, currentD, erreurD, toleranceD, pD;
    private int v = 0;
    public static PidRBL rotattion_Pid = new PidRBL(rotation_P, rotation_I, rotation_D);
    private final Telemetry telemetry;
    private IMU imu;
    private ALDNC_container robot;
    private double targetPosition;
    private double erreurPos;

    public Drive_Train(HardwareMap hmap, Telemetry tele, double x, double y, ALDNC_container roBot) {
        robot = roBot;

        xOdo = x;
        yOdo = y;
        telemetry = tele;

        left_drive = hmap.get(DcMotorEx.class, LEFT_MOTOR);
        right_drive = hmap.get(DcMotorEx.class, RIGHT_MOTOR);

        /*encoderG = hmap.get(DcMotorEx.class, ENCODEURG);
        encoderD = hmap.get(DcMotorEx.class, ENCODERD);*/

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_drive.setDirection(DcMotorSimple.Direction.FORWARD);
        right_drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        /*encoderD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderG.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        rotattion_Pid.SetTolerance(0.01);
        rotattion_Pid.SetInputLimits(0, Math.PI * 2);
        rotattion_Pid.SetContinuous(true);
        imu = hmap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();


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

    public void backward() {
        left_motor_power = -1;
        right_motor_power = -1;
    }

    public void stop() {
        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    public void drive(double forward, double turn) {
        left_motor_power = forward + turn;
        right_motor_power = forward - turn;

    }

    public void align_rotation(double target_angle, double real_angle, Telemetry telemetry) {
        double turn = rotattion_Pid.Calculate(target_angle * Math.PI / 180, real_angle);
        telemetry.addData("left motor power", turn);
        telemetry.addData("reight motor power", -turn);
        telemetry.addLine(rotattion_Pid.GetState());
    }

    public void slow_down(double ralentisseur) {
        left_motor_power /= ralentisseur;
        right_motor_power /= ralentisseur;
    }

    private void calculateValuesEncoderDetG() {
        valueEncoderD = right_drive.getCurrentPosition() - vielleValueD;
        valueEncoderG = left_drive.getCurrentPosition() - vielleValueG;
        vielleValueD = right_drive.getCurrentPosition();
        vielleValueG = left_drive.getCurrentPosition();
    }
    public double getHeadingRadians() {
        if (imu == null) return 0;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private void calculateDistanceGetD() {
        DG = Math.PI * 2 * rayon / CPR;
        DG = DG * valueEncoderG;
        DD = Math.PI * 2 * rayon / CPR;
        DD = DD * valueEncoderD;
    }

    private void calculateAngleRadiant() {
        //angle = (DD - DG)/DL;
        vieuxAngle = -getHeadingRadians();
        angleDegrees = Math.toDegrees(vieuxAngle);
        h = (DG + DD) / 2;
    }

    private void calculateXY() {
        x = Math.sin(vieuxAngle) * h;
        x = -x;
        y = Math.cos(vieuxAngle) * h;
        vieuxX += x;
        vieuxY += y;
    }

    private void telemetrieOdometrie() {
        telemetry.addData("x", vieuxX);
        telemetry.addData("y", vieuxY);
        telemetry.addData("angle", angleDegrees);
        telemetry.addData("jambe gauche", left_drive.getCurrentPosition());
        telemetry.addData("jambe droite", right_drive.getCurrentPosition());
        telemetry.addLine("truc de PID");
        telemetry.addData("target", targetDegrees);
    }

    public void odometrie() {
        calculateValuesEncoderDetG();
        calculateDistanceGetD();
        calculateAngleRadiant();
        calculateXY();
        telemetrieOdometrie();
    }

    public double Get_right_current() {
        return right_drive.getCurrentPosition();
    }

    public double Get_left_current() {
        return left_drive.getCurrentPosition();
    }

    public double Get_right_power() {
        return right_motor_power;
    }

    public double Get_left_power() {
        return left_motor_power;
    }

    public double getAngle() {
        return vieuxAngle;
    }
    private void goAngle(double x, double y) {
        if(erreur == 0){
            return;
        }
        right_motor_power = erreur * p;
        left_motor_power = -right_motor_power;
    }
    private void calculAngle(double x, double y ){
        target = Math.atan(y / x);
        targetDegrees = Math.toDegrees(target);
        erreur = targetDegrees - angleDegrees;

        if(Utils.IsInRange(angleDegrees, targetDegrees, tolerance_go_angle)){
            erreur = 0;
        }
    }

    public void goPos(double x, double y){
        targetD = Math.sqrt(x*x + y*y);
        if(v == 0){
            currentD = 0;
            v = 1;
        }
        currentD += DD;
        erreurD = targetD-currentD;
        if(Utils.IsInRange(currentD, targetD, toleranceD)){
            erreurD = 0;
            v = 0;
        }
        if (erreurD == 0)
            return;
        right_motor_power = erreurD*pD;
        left_motor_power = -right_motor_power;
    }

    @Override
    public void periodic() {
        //odometrie();

        //targetPosition = robot.Camera().getBearing()*ff_rotation;
        //targetPosition = targetPosition + Get_right_current();
        //erreurPos = targetPosition-Get_right_current();
        //if(erreurPos>-tolerence_rotation && erreurPos<tolerence_rotation){
        //    erreurPos = 0;
        //}
        //if(robot.Shooter().Is_Shooting()){
        //    drive(0, Math.max(-0.3, Math.min(0.3, erreurPos*p_rotation)));
        //    if(right_motor_power > seuilDriveShooter || right_motor_power < -seuilDriveShooter || left_motor_power > seuilDriveShooter || left_motor_power < -seuilDriveShooter){
        //        drive(robot.Left_joystick().getX()/1.3, robot.Right_joystick().getY()/1.3);
        //    }
//
        //}else if (robot.is_inTeleop){
        //    drive(robot.Left_joystick().getX(), robot.Right_joystick().getY());
        //}
        //drive(robot.Left_joystick().getX(), robot.Right_joystick().getY()); //default
        left_drive.setPower(left_motor_power);
        right_drive.setPower(right_motor_power);

    }
}



