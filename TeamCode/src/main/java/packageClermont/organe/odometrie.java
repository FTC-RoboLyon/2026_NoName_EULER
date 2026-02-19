package packageClermont.organe;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class odometrie {
    public IMU imu;
    private double x, y, valueEncoderD, valueEncoderG, vielleValueD, vielleValueG, DG, DD,rayon = 5.7, CPR = 580, vieuxX, vieuxY, vieuxAngle, h, angleDegrees, targetDegrees, target, right_motor_power, left_motor_power, p = 0.005, erreur;
    DcMotorEx jambe_droite;
    DcMotorEx jambe_gauche;
    Telemetry telemetry;
    public odometrie(double x, double y, Telemetry telemetry, DcMotorEx jambe_droite, DcMotorEx jambe_gauche, IMU imu){
        jambe_droite.setDirection(DcMotorSimple.Direction.FORWARD);
        jambe_droite.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jambe_droite.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        jambe_droite.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        jambe_gauche.setDirection(DcMotorSimple.Direction.REVERSE);
        jambe_gauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jambe_gauche.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        jambe_gauche.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.imu = imu;
        this.jambe_droite = jambe_droite;
        this.jambe_gauche = jambe_gauche;
        this.telemetry = telemetry;
        this.x = x;
        this.y = y;
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
    }
    public double getHeadingRadians() {
        if (imu == null) return 0;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    private void calculateValuesEncoderDetG() {
        valueEncoderD = jambe_droite.getCurrentPosition() - vielleValueD;
        valueEncoderG = jambe_gauche.getCurrentPosition() - vielleValueG;
        vielleValueD = jambe_droite.getCurrentPosition();
        vielleValueG = jambe_gauche.getCurrentPosition();
    }

    private void calculateDistanceGetD() {
        DG = Math.PI * 2 * rayon / CPR;
        DG = DG * valueEncoderG;
        DD = Math.PI * 2 * rayon / CPR;
        DD = DD * valueEncoderD;
    }

    private void calculateAngleRadiant() {
        //angle = (DD - DG)/DL;
        vieuxAngle = getHeadingRadians();
        angleDegrees = Math.toDegrees(vieuxAngle);
        h = (DG + DD) / 2;
    }

    private void calculateXY() {
        x = Math.sin(vieuxAngle) * h;
        y = Math.cos(vieuxAngle) * h;
        vieuxX += x;
        vieuxY += y;
    }

    private void telemetrieOdometrie() {
        telemetry.addData("x", vieuxX);
        telemetry.addData("y", vieuxY);
        telemetry.addData("angle", angleDegrees);
        telemetry.addData("jambe gauche", jambe_gauche.getCurrentPosition());
        telemetry.addData("jambe droite", jambe_droite.getCurrentPosition());
        telemetry.addLine("truc de PID");
        telemetry.addData("target", targetDegrees);
        telemetry.addData("powerR", right_motor_power);
        telemetry.addData("powerL", left_motor_power);
    }
    public void goPos(double x, double y) {
        target = Math.atan(y / x);
        targetDegrees = Math.toDegrees(target);
        erreur = targetDegrees - angleDegrees;
        right_motor_power = erreur * p;
        left_motor_power = -right_motor_power;
    }

    public void odometrie(double x, double y) {
        calculateValuesEncoderDetG();
        calculateDistanceGetD();
        calculateAngleRadiant();
        calculateXY();
        telemetrieOdometrie();
    }

}
