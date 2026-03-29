package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.FEEDER_SERVO;
import static org.firstinspires.ftc.teamcode.euler.Constant.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.SHOOTER_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.VISEUR_SERVO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.euler.driver.Driver;
import org.firstinspires.ftc.teamcode.euler.feeder.Feeder;
import org.firstinspires.ftc.teamcode.euler.intake.Intake;
import org.firstinspires.ftc.teamcode.euler.shooter.Shooter;
import org.firstinspires.ftc.teamcode.euler.utils.ButtonReader;
import org.firstinspires.ftc.teamcode.euler.viseur.Viseur;

/**
 * EulerTeleop - OpMode principal.
 * Version avec Feeder simplifié à 2 positions (Haut/Bas).
 */
@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends LinearOpMode {
    private Driver myDriver;
    private Intake myIntake;
    private Shooter myShooter;
    private Viseur myViseur;
    private Feeder myFeeder;

    private ButtonReader btnA;
    private ButtonReader btnB;
    private ButtonReader btnX;
    private ButtonReader btnL_Bumper;
    private ButtonReader btnL_Trigger;
    private ButtonReader btnR_Bumper;

    void initialize() {
        myDriver = new Driver(hardwareMap.get(DcMotor.class, LEFT_MOTOR), hardwareMap.get(DcMotor.class, RIGHT_MOTOR));
        myIntake = new Intake(hardwareMap.get(DcMotor.class, INTAKE_MOTOR));
        myShooter = new Shooter(hardwareMap.get(DcMotor.class, SHOOTER_MOTOR));
        myViseur = new Viseur(hardwareMap.get(Servo.class, VISEUR_SERVO));
        myFeeder = new Feeder(hardwareMap.get(Servo.class, FEEDER_SERVO));

        btnA = new ButtonReader(() -> gamepad1.a);
        btnB = new ButtonReader(() -> gamepad1.b);
        btnX = new ButtonReader(() -> gamepad1.x);
        btnL_Bumper = new ButtonReader(() -> gamepad1.left_bumper);
        btnL_Trigger = new ButtonReader(() -> gamepad1.left_trigger > 0.5);
        btnR_Bumper = new ButtonReader(() -> gamepad1.right_bumper);
    }

    @Override
    public void runOpMode() {
        initialize();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. COMMANDES (INTENTIONS)

            // Shooter & Viseur
            if (btnA.wasJustPressed()) {
                myShooter.toggleShootNear();
                myViseur.aimNear();
            } else if (btnB.wasJustPressed()) {
                myShooter.toggleShootMiddle();
                myViseur.aimMiddle();
            } else if (btnX.wasJustPressed()) {
                myShooter.toggleShootFar();
                myViseur.aimFar();
            }

            // Feeder : Action simple Toggle (Haut / Bas)
            if (btnR_Bumper.wasJustPressed()) {
                myFeeder.toggle();
            }

            // Intake
            if (btnL_Bumper.wasJustPressed()) myIntake.toggleCollect();
            if (btnL_Trigger.wasJustPressed()) myIntake.toggleEject();

            // Pilotage
            myDriver.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

            // 2. ACTIONS (RÉALITÉ)
            myDriver.update();
            myIntake.update();
            myShooter.update();
            myViseur.update();
            myFeeder.update();

            // 3. TÉLÉMÉTRIE
            telemetry.addLine("--- SYSTÈMES ---");
            telemetry.addData("Châssis", myDriver.getState());
            telemetry.addData("Intake", myIntake.getState());

            telemetry.addLine("--- SHOOTER / VISEUR / FEEDER ---");
            telemetry.addData("Shooter", myShooter.getState() + " - " + myShooter.getTargetState());
            telemetry.addData("Prêt ?", myShooter.isReady() ? "[OUI]" : "Ajustement...");
            telemetry.addData("Viseur", myViseur.getState() + " - " + myViseur.getTargetState());
            telemetry.addData("Feeder", myFeeder.getState() + " - " + myFeeder.getTargetState());

            telemetry.update();
        }
    }
}
