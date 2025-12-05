package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.INTAKE;
import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.SHOOTER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.euler.Driver;

import java.util.Objects;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor = hardwareMap.get(DcMotor.class, LEFT_MOTOR);
        DcMotor right_motor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE);
        DcMotor shooter = hardwareMap.get(DcMotor.class, SHOOTER);
        Servo feeder = hardwareMap.get(CRServo.class, FEEDER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Driver myRobotDriver = new Driver(left_motor, right_motor, intake, shooter, feeder);

        int puissanceIntake = 1;
        int velocityShooter = 5100;
        int velocityShooterPos1 = 0;
        int velocityShooterPos2 = 0;
        int variableInverseurIntake = 0;
        int variableInverseurShooter = 0;
        boolean right_bumper = gamepad1.right_bumper;
        boolean left_bumper = gamepad1.left_bumper;
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean fleche_haut = gamepad1.dpad_up;
        boolean fleche_bas = gamepad1.dpad_down;
        boolean Fgauche = gamepad1.dpad_left;
        boolean Fdroite = gamepad1.dpad_right;





        while (opModeIsActive()) {
            float turn = gamepad1.left_stick_x;
            float forward = -gamepad1.right_stick_y;
            float valueLeftMotor = forward - turn;
            float valueRightMotor = forward + turn;

            telemetry.addData("Gamepad", "left:" + turn);
            telemetry.addData("Gamepad", "right:" + forward);
            telemetry.addData("Puissance Shooter =", "velocityShooter");
            telemetry.update();

            //myRobotDriver.limitateur(valueLeftMotor, valueRightMotor);
            myRobotDriver.drive(valueLeftMotor, valueRightMotor);
            //myRobotDriver.inverseurIntake(puissanceIntake, variableInverseurIntake);
            myRobotDriver.intake(puissanceIntake, left_bumper);
            //myRobotDriver.Inverseurshooter(puissanceShooter, variableInverseurShooter);
            //myRobotDriver.règleurPuissanceShooter(velocitysooter);
            //myRobotDriver.positionsShooter(velocityShooter, puissanceShooterPos1, puissanceShooterPos2);
            myRobotDriver.shooter(velocityShooter, right_bumper);
            myRobotDriver.feeder(gamepad1.xWasPressed(), gamepad1.xWasReleased());
        }
    }
}


//Ce qui est en commentaire est à mettre apres avoir déjà regardé si le shooter et l'intake marchent
//Parce que sinon ça encombre le code et si y'a des erreurs, on saura pas d'où elles viennent