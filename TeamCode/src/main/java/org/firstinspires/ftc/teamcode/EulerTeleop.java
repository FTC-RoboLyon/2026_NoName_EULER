package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.euler.Driver;

@TeleOp(name = "EulerTeleop",group = "Euler")
public class EulerTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Driver myRobotDriver = new Driver(leftMotor, rightMotor);

        while (opModeIsActive()) {
            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;

            telemetry.addData("Gamepad", "left:"+left);
            telemetry.addData("Gamepad", "right:"+right);
            telemetry.update();

            myRobotDriver.drive(left, right);
        }
    }
}
