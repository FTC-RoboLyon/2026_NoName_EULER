package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constant.FEEDER;
import static org.firstinspires.ftc.teamcode.Constant.INTAKE;
import static org.firstinspires.ftc.teamcode.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Constant.SHOOTER;
import static org.firstinspires.ftc.teamcode.Constant.VISEUR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleopForTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(Servo.class, FEEDER);
        Servo viseur = hardwareMap.get(Servo.class, VISEUR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Driver myRobotDriver = new Driver(left_motor, right_motor, intake, shooter, feeder, viseur);

        int puissanceIntake = 1;
        int velocityShooter = 5100;
        int velocityShooterPos1 = 0;
        int velocityShooterPos2 = 0;
        double posviseur = 0.6;
        double posviseur_bank = 0.6;
        int velocity_bank = 900;
        double posviseur_far = 0.4;
        int velocity_far = 1600;


        feeder.setPosition(0);

        while (opModeIsActive()) {
            float turn = gamepad1.right_stick_x;
            float forward = -gamepad1.left_stick_y;
            float valueLeftMotor = forward + turn;
            float valueRightMotor = forward - turn;

            telemetry.addData("Puissance Shooter =", velocityShooter);
            telemetry.addData("Position Viseur ", viseur.getPosition());
            telemetry.update();


            // se deplacer
            myRobotDriver.drive(valueLeftMotor, valueRightMotor, gamepad2.b);

            //l'intake
            myRobotDriver.intake(puissanceIntake, gamepad1.left_bumper, gamepad1.left_trigger);

            //régler la velocité du shooter
            velocityShooter = myRobotDriver.regleurPuissanceShooter(velocityShooter, velocity_bank, velocity_far, gamepad1.dpadUpWasPressed(), gamepad1.dpadDownWasPressed(), gamepad2.b, gamepad2.y);
            myRobotDriver.shooter(velocityShooter, gamepad1.right_bumper, gamepad1.right_trigger);

            //Le feeder
            myRobotDriver.feeder(gamepad1.xWasPressed(), gamepad1.xWasReleased());

            //Regler le viseur
            posviseur = myRobotDriver.viseur(gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.dpadLeftWasPressed(), gamepad1.dpadRightWasPressed(), posviseur, posviseur_far, posviseur_bank);
            viseur.setPosition(posviseur);
        }
    }
}


