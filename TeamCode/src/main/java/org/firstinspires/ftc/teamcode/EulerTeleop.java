package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.SHOOTER_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.euler.driver.Driver;
import org.firstinspires.ftc.teamcode.euler.intake.Intake;
import org.firstinspires.ftc.teamcode.euler.shooter.Shooter;

@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends LinearOpMode {
    Driver myDriver;
    Intake myIntake;
    Shooter myShooter;

    void initialize() {
        myDriver = new Driver(hardwareMap.get(DcMotor.class, LEFT_MOTOR), hardwareMap.get(DcMotor.class, RIGHT_MOTOR));
        myIntake = new Intake(hardwareMap.get(DcMotor.class, INTAKE_MOTOR));
        myShooter = new Shooter(hardwareMap.get(DcMotor.class, SHOOTER_MOTOR));
    }

    @Override
    public void runOpMode() {
        initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Shooter & Viseur : règle position viseur + vélocité shooter
            boolean shootNear = gamepad1.a;
            boolean shootMiddle = gamepad1.b;
            boolean shootFar = gamepad1.x;

            if (gamepad1.a) {
                myShooter.toggleShootNear();
                myViseur.toggleAimNear();
            } else if (gamepad1.b) {
                myShooter.toggleShootMiddle();
                myViseur.toggleAimMiddle();
            } else if (gamepad1.x) {
                myShooter.toggleShootFar();
                myViseur.toggleAimFar();
            }

            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;
            myDriver.drive(left, right);

            telemetry.addData("DriverState", myDriver.getState());
            telemetry.update();

            // attention à l'appui trop long ...
            // si besoin il faudra avoir un etat precedent et gerer en fonction
            if (gamepad1.left_bumper) {
                myIntake.toggleCollect();
            }
            if (gamepad1.left_trigger_pressed) {
                myIntake.toggleEject();
            }
        }
    }
}
