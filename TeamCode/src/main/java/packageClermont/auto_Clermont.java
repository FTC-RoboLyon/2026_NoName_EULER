package packageClermont;

import static ALDNC_organe.Constant.FEEDER;
import static ALDNC_organe.Constant.INTAKE;
import static ALDNC_organe.Constant.LEFT_MOTOR;
import static ALDNC_organe.Constant.RIGHT_MOTOR;
import static ALDNC_organe.Constant.SHOOTER;
import static ALDNC_organe.Constant.VISEUR;

import ALDNC_organe.ALDNC_auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class auto_Clermont extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double kp = 20;
        double ki = 5;
        double kd = 10;
        double kf = 10;


        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        PIDFCoefficients pidf = new PIDFCoefficients(kp, ki, kd, kf);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);


        waitForStart();
        left_motor.setPower(-0.5);
        right_motor.setPower(-0.5);
        sleep(500);
        left_motor.setPower(0);
        right_motor.setPower(0);
        shooter.setVelocity(2300);
        sleep(3000);
        feeder.setPosition(0);
        viseur.setPosition(1);
        for(int a = 0; a<4;a++){
            feeder.setPosition(1);
            sleep(300);
            feeder.setPosition(0);
            sleep(2000);
        }


        if (opModeIsActive()){
            viseur.setPosition(1);
            right_motor.setPower(-0.5);
            left_motor.setPower(-1);
            sleep(2000);
            left_motor.setPower(0);
            right_motor.setPower(0);

        }
    }

}
