package FRC_ALDNC.Auto.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

import FRC_ALDNC.Auto.Constant;
public class DriveSubsystem extends SubsystemBase {
    private double left_motor_power, right_motor_power, valueEncoderD, valueEncoderG, vielleValueD, vielleValueG, vieuxX, vieuxY, vieuxAngle,  angleDegrees, DG, DD, h;
    public double  x, y, angle, forward, turn;
    private double erreurAngle, pAngle = 0.7, targetAngle, pDistance = 0.01, erreurDistance, valeurQuOnVaRajouterAuxValeursDuTurnPourQuIlAvanceVersLaTarget;
    double CPR = 8192,diametre = 7.27,DL = 16.21;

    private final
    DcMotorEx motorRight, motorLeft;
    HardwareMap hmap;
    Telemetry telemetry;
    public DriveSubsystem(HardwareMap hmap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hmap = hmap;
        motorRight = hmap.get(DcMotorEx.class, Constant.RIGHT_MOTOR);
        motorLeft = hmap.get(DcMotorEx.class, Constant.LEFT_MOTOR);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    private void calculateValuesEncoderDetG() {
        valueEncoderD = motorRight.getCurrentPosition() - vielleValueD;
        valueEncoderG = motorLeft.getCurrentPosition() - vielleValueG;
        vielleValueD = motorRight.getCurrentPosition();
        vielleValueG = motorLeft.getCurrentPosition();
    }
    private void calculateDistanceGetD() {
        DG = Math.PI * diametre / CPR;
        DG = DG * valueEncoderG;
        DD = Math.PI * diametre / CPR;
        DD = DD * valueEncoderD;
    }
    private void calculateAngleRadiant() {
        angle = (DD - DG)/DL;
        vieuxAngle += angle;
        if (vieuxAngle > Math.PI){
            vieuxAngle -= 2*Math.PI;
        }
        else if (vieuxAngle < -Math.PI){
            vieuxAngle += 2*Math.PI;
        }
        angleDegrees = Math.toDegrees(vieuxAngle);
        h = (DG + DD) / 2;
    }
    private void calculateXY() {
        x = Math.cos(vieuxAngle) * h;
        y = Math.sin(vieuxAngle) * h;
        vieuxX += x;
        vieuxY += y;
    }
    private void telemetrieOdometrie() {
        telemetry.addData("x", vieuxX);
        telemetry.addData("y", vieuxY);
        telemetry.addData("angle", angleDegrees);
        telemetry.addData("targetAngle", targetAngle);
        telemetry.update();
    }
    public void odometrie(){
        calculateValuesEncoderDetG();
        calculateDistanceGetD();
        calculateAngleRadiant();
        calculateXY();
        telemetrieOdometrie();
    }
    public void drive(Gamepad gamepad1){
        forward = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        right_motor_power = forward-turn;
        left_motor_power = forward+turn;
    }
    public void goAngle(double x, double y){
        y -= vieuxY;
        x -= vieuxX;
        targetAngle = Math.atan2(y ,x);
        erreurAngle = targetAngle - vieuxAngle;
        if(erreurAngle > Math.PI)
            erreurAngle -= 2*Math.PI;

        if(erreurAngle < -Math.PI)
            erreurAngle += 2*Math.PI;
        right_motor_power = erreurAngle*pAngle;
        left_motor_power = -right_motor_power;
    }
    public void goPos(double x, double y){
        x -= vieuxX;
        y -= vieuxY;
        erreurDistance = Math.sqrt(x*x+y*y);
        valeurQuOnVaRajouterAuxValeursDuTurnPourQuIlAvanceVersLaTarget = erreurDistance*pDistance;
        if(valeurQuOnVaRajouterAuxValeursDuTurnPourQuIlAvanceVersLaTarget > 0.8){
            valeurQuOnVaRajouterAuxValeursDuTurnPourQuIlAvanceVersLaTarget = 0.8;
        }// je la bloque à 0.8 parce que si elle est tout le temps à 1, ca va écraser les valeurs du turn et il tournera juste pas
        // en plus avec notre roue de gauche qui va à 2 à l'heure c'est grave important
        right_motor_power += valeurQuOnVaRajouterAuxValeursDuTurnPourQuIlAvanceVersLaTarget;
        left_motor_power += right_motor_power;
    }
    @Override
    public void periodic(){
        odometrie();
        motorRight.setPower(right_motor_power);
        motorLeft.setPower(left_motor_power);
    }
}
