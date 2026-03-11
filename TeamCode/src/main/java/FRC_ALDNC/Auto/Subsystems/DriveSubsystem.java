package FRC_ALDNC.Auto.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Constant;
public class DriveSubsystem extends SubsystemBase {
    private double left_motor_power, right_motor_power, x, y, valueEncoderD, valueEncoderG, vielleValueD, vielleValueG, vieuxX, vieuxY, vieuxAngle, angle, angleDegrees, rayon = 5.7, CPR = 8196, DG, DD, DL = 36, h;

    DcMotorEx motorRight, motorLeft;
    HardwareMap hmap;
    Telemetry telemetry;
    public DriveSubsystem(HardwareMap hmap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hmap = hmap;
        motorRight = hmap.get(DcMotorEx.class, Constant.RIGHT_MOTOR);
        motorLeft = hmap.get(DcMotorEx.class, Constant.LEFT_MOTOR);

        motorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        motorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    private void calculateValuesEncoderDetG() {
        valueEncoderD = motorRight.getCurrentPosition() - vielleValueD;
        valueEncoderG = motorLeft.getCurrentPosition() - vielleValueG;
        vielleValueD = motorRight.getCurrentPosition();
        vielleValueG = motorLeft.getCurrentPosition();
    }
    private void calculateDistanceGetD() {
        DG = Math.PI * 2 * rayon / CPR;
        DG = DG * valueEncoderG;
        DD = Math.PI * 2 * rayon / CPR;
        DD = DD * valueEncoderD;
    }
    private void calculateAngleRadiant() {
        angle = (DD - DG)/DL;
        vieuxAngle += angle;
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
        telemetry.addData("jambe gauche", motorLeft.getCurrentPosition());
        telemetry.addData("jambe droite", motorRight.getCurrentPosition());
    }
    public void odometrie(){
        calculateValuesEncoderDetG();
        calculateDistanceGetD();
        calculateAngleRadiant();
        calculateXY();
        telemetrieOdometrie();
    }
    public void drive(double forward, double turn){
        right_motor_power = forward+turn;
        left_motor_power = forward-turn;
    }
    @Override
    public void periodic(){
        odometrie();
        motorRight.setPower(right_motor_power);
        motorLeft.setPower(left_motor_power);
    }
}
