package FRC_ALDNC.Auto.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

import FRC_ALDNC.Auto.Constant;
@Config
public class DriveSubsystem extends SubsystemBase {
    private double  valueEncoderD, valueEncoderG, vielleValueD, vielleValueG, vieuxX, vieuxY, vieuxAngle,angleAncienneLoop,  angleDegrees, DG, DD, h;
    public double  x, y, angle, forward, turn, left_motor_power, right_motor_power;
    public static double erreurAngle, pAngle = 1.7, targetAngle, pDistance = 0.07;

    double CPR = 8192,diametre = 7.27,DL = 16.21;

    private final
    DcMotorEx motorRight, motorLeft;
    HardwareMap hmap;
    Telemetry telemetry;

    NavXSubsystem navx;
    public DriveSubsystem(NavXSubsystem navx, HardwareMap hmap, Telemetry telemetry, double xDepart, double yDepart, double angleDepart){
        this.navx = navx;
        vieuxAngle = angleDepart;
        vieuxX = xDepart;
        vieuxY = yDepart;
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

        angleAncienneLoop = navx.getAngle();
    }
    public void navxInit(){
        navx.reset_A_MettreDans_L_Init();
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
        vieuxAngle = navx.getAngle();
        if(vieuxAngle > Math.PI){
            vieuxAngle -= 2*Math.PI;
        }
        if(vieuxAngle < -Math.PI){
            vieuxAngle += 2*Math.PI;
        }
        angleDegrees = Math.toDegrees(vieuxAngle);
        h = (DG + DD) / 2;
    }

    public double getAngle(){
        return vieuxAngle;
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
        telemetry.addData("targetAngle", Math.toDegrees(targetAngle));
        telemetry.update();
    }
    public void odometrie(){
        calculateValuesEncoderDetG();
        calculateDistanceGetD();
        calculateAngleRadiant();
        calculateXY();
        telemetrieOdometrie();
    }
    public void goAngle(double x, double y){
        y -= vieuxY;
        x -= vieuxX;
        targetAngle = Math.atan2(y ,x);
        erreurAngle = targetAngle - vieuxAngle;
        if(erreurAngle > Math.PI)
            erreurAngle -= 2*Math.PI;
        else if(erreurAngle < -Math.PI)
            erreurAngle += 2*Math.PI;
        double turn = erreurAngle*pAngle;
        if(turn > 0.5)turn = 0.5;
        if(turn < -0.5)turn = -0.5;
        right_motor_power = turn;
        left_motor_power = -right_motor_power;
    }
    public void goPos(double targetX, double targetY){
        double dx = targetX - vieuxX;
        double dy = targetY - vieuxY;

        double distance = Math.sqrt(dx*dx + dy*dy);

        double targetAngleToPos = Math.atan2(dy, dx);

        double angleDiff = targetAngleToPos - vieuxAngle;
        while(angleDiff > Math.PI) angleDiff -= 2*Math.PI;
        while(angleDiff < -Math.PI) angleDiff += 2*Math.PI;

        double directionMultiplier = 1.0;
        if(Math.abs(angleDiff) > Math.PI/2){
            directionMultiplier = -1.0;
            angleDiff = angleDiff > 0 ? angleDiff - Math.PI : angleDiff + Math.PI;
        }

        double vitesseDistance = distance * pDistance;
        if(vitesseDistance > 0.5) vitesseDistance = 0.5;

        double turn = angleDiff * pAngle;
        if(turn > 0.3)turn = 0.3;
        if(turn < -0.3) turn = -0.3;
        double turnDroite = turn < 0 ? 0 : turn;
        double turnGauche = -turn < 0 ? 0 : -turn;

        right_motor_power = directionMultiplier * vitesseDistance + turnDroite;
        left_motor_power  = directionMultiplier * vitesseDistance + turnGauche;
    }
    public double getDistanceTo(double targetX, double targetY){
        double dx = targetX - vieuxX;
        double dy = targetY - vieuxY;
        return Math.sqrt(dx*dx + dy*dy);
    }
    public double getAngleTo(){
        return erreurAngle;
    }
    public void drive(double forward, double turn){
        right_motor_power = -forward-turn;
        left_motor_power = -forward+turn;
    }
    @Override
    public void periodic(){
        odometrie();
        motorRight.setPower(right_motor_power);
        motorLeft.setPower(left_motor_power);
    }
}
