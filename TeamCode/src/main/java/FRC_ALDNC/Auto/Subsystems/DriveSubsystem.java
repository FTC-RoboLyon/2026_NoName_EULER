package FRC_ALDNC.Auto.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import FRC_ALDNC.Auto.Constant;
public class DriveSubsystem extends SubsystemBase {
    private double left_motor_power, right_motor_power, valueEncoderD, valueEncoderG, vielleValueD, vielleValueG, vieuxX, vieuxY, vieuxAngle,  angleDegrees, DG, DD, h;
    public double  x, y, angle, forward, turn;
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
        double CPR = 8192;
        double diametre = 7.27;
        DG = Math.PI * diametre / CPR;
        DG = DG * valueEncoderG;
        DD = Math.PI * diametre / CPR;
        DD = DD * valueEncoderD;
    }
    private void calculateAngleRadiant() {
        double DL = 36;
        angle = (DD - DG)/DL;
        vieuxAngle += angle;
        if (vieuxAngle > Math.PI){
            vieuxAngle = -(vieuxAngle-(vieuxAngle - Math.PI));
        } else if (vieuxAngle < -Math.PI) {
            vieuxAngle = -(vieuxAngle-(Math.PI - vieuxAngle));
        }
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
    @Override
    public void periodic(){
        odometrie();
        motorRight.setPower(right_motor_power);
        motorLeft.setPower(left_motor_power);
    }
}
