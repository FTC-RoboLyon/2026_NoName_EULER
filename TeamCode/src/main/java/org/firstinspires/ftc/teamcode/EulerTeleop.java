package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.euler.Driver;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Driver myRobotDriver = new Driver(left_motor, right_motor);

        while (opModeIsActive()) {
            float turn = gamepad1.left_stick_x;
            float forward = -gamepad1.right_stick_y;

            telemetry.addData("Gamepad", "left:" + turn);
            telemetry.addData("Gamepad", "right:" + forward);
            telemetry.update();

            myRobotDriver.drive(turn, forward);
        }
    }
}
