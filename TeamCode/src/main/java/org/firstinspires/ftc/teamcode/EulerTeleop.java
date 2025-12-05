package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.INTAKE;
import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.SHOOTER;

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
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Driver myRobotDriver = new Driver(left_motor, right_motor, intake, shooter);
        int puissanceIntake = 1;
        int vélocitéShooter = 0;
        int variableInverseurIntake = 0;
        int variableInverseurShooter = 0;
        int puissanceShooterPos1 = 0;
        int puissanceShooterPos2 = 0;

        while (opModeIsActive()) {
            float turn = gamepad1.left_stick_x;
            float forward = -gamepad1.right_stick_y;
            float valueLeftMotor = forward - turn;
            float valueRightMotor = forward + turn;

            telemetry.addData("Gamepad", "left:" + turn);
            telemetry.addData("Gamepad", "right:" + forward);
            telemetry.addData("Puissance Shooter =", vélocitéShooter);
            telemetry.update();

            myRobotDriver.intake(puissanceIntake, gamepad1.left_bumper);
            myRobotDriver.shooter(vélocitéShooter, gamepad1.right_bumper);
            myRobotDriver.drive(valueLeftMotor, valueRightMotor);
            //myRobotDriver.limitateur(valueLeftMotor, valueRightMotor);
            //myRobotDriver.inverseurIntake(puissanceIntake, variableInverseurIntake);
            //myRobotDriver.Inverseurshooter(vélocitéShooter, variableInverseurShooter);
            myRobotDriver.règleurPuissanceShooter(vélocitéShooter, gamepad1.dpad_up, gamepad1.dpad_down);
            myRobotDriver.positionsShooter(vélocitéShooter, puissanceShooterPos1, puissanceShooterPos2, gamepad1.dpad_left, gamepad1.dpad_right);

        }
    }
}


//Ce qui est en commentaire est à mettre apres avoir déjà regardé si le shooter et l'intake marchent
//Parce que sinon ça encombre le code et si y'a des erreurs, on saura pas d'où elles viennent