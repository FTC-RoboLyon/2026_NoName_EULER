package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.euler.Driver;

@Autonomous(preselectTeleOp = "")
public class EulerAutonomous extends LinearOpMode {
    Driver driver;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        driver = new Driver(leftMotor, rightMotor);

    }
}
