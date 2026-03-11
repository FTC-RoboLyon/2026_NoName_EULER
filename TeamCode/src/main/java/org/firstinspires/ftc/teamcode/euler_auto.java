package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Constant.FEEDER;
import static org.firstinspires.ftc.teamcode.Constant.SHOOTER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(preselectTeleOp = "EulerAuto",  group = "Euler")
public class euler_auto extends LinearOpMode {

    @Override
    public void runOpMode () throws InterruptedException {
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        int velocité = 1200;
        waitForStart();

        if (opModeIsActive()){
            feeder.setPosition(0);
            ((DcMotorEx) shooter).setVelocity(velocité);
            sleep(3000);
            feeder.setPosition(0.2);
            sleep(500);
            feeder.setPosition(0);
            sleep(3000);
            feeder.setPosition(0.2);
            sleep(500);
            feeder.setPosition(0);

          }
    }
}