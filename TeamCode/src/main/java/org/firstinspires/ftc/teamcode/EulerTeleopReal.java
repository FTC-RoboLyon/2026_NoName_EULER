package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constant.COMPTEUR_BALLE;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleopReal extends LinearOpMode {

private IMU imu;
private VoltageSensor ControlHub_VoltageSensor;



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);
        imu = hardwareMap.get(IMU.class, "imu");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DistanceSensor compteurBalle = hardwareMap.get(DistanceSensor.class, COMPTEUR_BALLE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        Driver myRobotDriver = new Driver(left_motor, right_motor, intake, shooter, feeder, viseur);

        IMU.Parameters imu_parameters;
        imu_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(0, 0, 90)));
        imu.initialize(imu_parameters);
        imu.resetYaw();


        double velocityShooter = 1;
        double posviseur = 0.6;
        double posviseur_bank = 0.6;
        double velocity_bank = 0.5;
        double posviseur_far = 0.37;
        double velocity_far = 1;
        double robotOrienDegrees;
        double real_velo = 0;
        float forward;
        float turn;
        double seuil_shootter = 12;
        double voltage;
        imu.resetYaw();
        feeder.setPosition(0);
        voltage = ControlHub_VoltageSensor.getVoltage();
        velocityShooter = (velocityShooter * seuil_shootter) / voltage;
        velocity_bank = (velocity_bank * seuil_shootter) / voltage;
        velocity_far = (velocity_far * seuil_shootter) / voltage;
        double distance = compteurBalle.getDistance(DistanceUnit.CM);
        boolean isIntaking = false;

        waitForStart();
        while (opModeIsActive()) {



            if (distance < 25){
                isIntaking = true;
            } else {
                isIntaking = false;
            }




            turn = gamepad1.right_stick_x;
            forward = -gamepad1.left_stick_y;

            // se deplacer
            myRobotDriver.drivePourDefit(turn);
            float valueLeftMotor = forward + turn;
            float valueRightMotor = forward - turn;
            myRobotDriver.drive(valueLeftMotor, valueRightMotor, gamepad1.dpad_up);

            //l'intake
            myRobotDriver.intake(gamepad1.left_bumper, gamepad1.left_trigger);

            //régler la velocité du shooter et le viseur
            if (gamepad1.bWasPressed()){
                velocityShooter = velocity_bank;
            } else if (gamepad1.yWasPressed()) {
                velocityShooter = velocity_far;
            }
            myRobotDriver.viseur(gamepad1.a, gamepad1.b, gamepad1.y,gamepad1.dpadRightWasPressed(), gamepad1.dpadLeftWasPressed(),  posviseur, posviseur_far, posviseur_bank);
            myRobotDriver.shooter(velocityShooter, gamepad1.right_bumper, gamepad1.right_trigger, real_velo);
            viseur.setPosition(posviseur);

            //Le feeder
            myRobotDriver.feeder(gamepad1.xWasPressed(), gamepad1.xWasReleased());


            real_velo = ((DcMotorEx) shooter).getVelocity();

            distance = compteurBalle.getDistance(DistanceUnit.CM);
            YawPitchRollAngles robotYawPitchRoll;
            robotYawPitchRoll = imu.getRobotYawPitchRollAngles();
            robotOrienDegrees = -robotYawPitchRoll.getYaw(AngleUnit.DEGREES);
            telemetry.addData("robot orientation", robotOrienDegrees);
            telemetry.addData("Velocité programmé Shooter =", velocityShooter);
            telemetry.addData("Vrai vélocité Shooter =", real_velo);
            telemetry.addData("Position Viseur ", viseur.getPosition());
            telemetry.addData("Distance", compteurBalle.getDistance(DistanceUnit.CM));
            telemetry.addData("is intaking", isIntaking);
            telemetry.update();


        }

    }
}