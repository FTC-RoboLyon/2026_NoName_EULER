package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constant.FEEDER;
import static org.firstinspires.ftc.teamcode.Constant.INTAKE;
import static org.firstinspires.ftc.teamcode.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Constant.SHOOTER;
import static org.firstinspires.ftc.teamcode.Constant.VISEUR;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleopReal extends LinearOpMode {

private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Driver myRobotDriver = new Driver(left_motor, right_motor, intake, shooter, feeder, viseur);

        IMU.Parameters imu_parameters;
        imu_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(0, 0, 90)));
        imu.initialize(imu_parameters);
        imu.resetYaw();

        int puissanceIntake = 1;
        int velocityShooter = 5100;
        int velocityShooterPos1 = 0;
        int velocityShooterPos2 = 0;
        double posviseur = 0.6;
        double posviseur_bank = 0.6;
        int velocity_bank = 1700;
        double posviseur_far = 0.4;
        int velocity_far = 1600;
        double robotOrienDegrees;
        double real_velo;
        imu.resetYaw();
        feeder.setPosition(0);

        while (opModeIsActive()) {
            YawPitchRollAngles robotYawPitchRoll;
            robotYawPitchRoll = imu.getRobotYawPitchRollAngles();
            robotOrienDegrees = -robotYawPitchRoll.getYaw(AngleUnit.DEGREES);
            telemetry.addData("robot orientation", robotOrienDegrees);

            real_velo = ((DcMotorEx) shooter).getVelocity();

            float turn = gamepad1.right_stick_x;
            float forward = -gamepad1.left_stick_y;


            telemetry.addData("Velocité programmé Shooter =", velocityShooter);
            telemetry.addData("Vrai vélocité Shooter =", real_velo);
            telemetry.addData("Position Viseur ", viseur.getPosition());
            telemetry.update();


            // se deplacer
            myRobotDriver.drivePourDefit(turn);
            float valueLeftMotor = forward + turn;
            float valueRightMotor = forward - turn;
            myRobotDriver.drive(valueLeftMotor, valueRightMotor, gamepad1.dpad_up);

            //l'intake
            myRobotDriver.intake(puissanceIntake, gamepad1.left_bumper, gamepad1.left_trigger);

            //régler la velocité du shooter et le viseur
            velocityShooter = myRobotDriver.regleurPuissanceShooter(velocityShooter, velocity_bank, velocity_far, gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed(), gamepad1.b, gamepad1.y);
            posviseur = myRobotDriver.viseur(gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.dpadLeftWasPressed(), gamepad1.dpadRightWasPressed(), posviseur, posviseur_far, posviseur_bank);
            myRobotDriver.shooter(velocityShooter, gamepad1.right_bumper, gamepad1.right_trigger, real_velo);
            viseur.setPosition(posviseur);

            //Le feeder
            myRobotDriver.feeder(gamepad1.xWasPressed(), gamepad1.xWasReleased());



        }

    }
}