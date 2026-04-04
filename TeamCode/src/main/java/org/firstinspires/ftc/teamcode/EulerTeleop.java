package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.euler.Constant.FEEDER_SERVO;
import static org.firstinspires.ftc.teamcode.euler.Constant.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.PATHER_SERVO;
import static org.firstinspires.ftc.teamcode.euler.Constant.RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.SHOOTER_MOTOR;
import static org.firstinspires.ftc.teamcode.euler.Constant.VISEUR_SERVO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.euler.driver.Driver;
import org.firstinspires.ftc.teamcode.euler.feeder.Feeder;
import org.firstinspires.ftc.teamcode.euler.intake.Intake;
import org.firstinspires.ftc.teamcode.euler.pather.Pather;
import org.firstinspires.ftc.teamcode.euler.shooter.Shooter;
import org.firstinspires.ftc.teamcode.euler.utils.ButtonReader;
import org.firstinspires.ftc.teamcode.euler.viseur.Viseur;

/**
 * EulerTeleop - OpMode principal.
 * Version finale avec architecture intention/action et gestion homogène des boutons.
 */
@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends LinearOpMode {
    private Driver myDriver;
    private Intake myIntake;
    private Shooter myShooter;
    private Viseur myViseur;
    private Feeder myFeeder;
    private Pather myPather;

    private ButtonReader btnA;
    private ButtonReader btnB;
    private ButtonReader btnX;
    private ButtonReader btnL_Bumper;
    private ButtonReader btnL_Trigger;
    private ButtonReader btnR_Bumper;
    private ButtonReader btnR_Trigger;

    void initialize() {
        // Hardware
        myDriver = new Driver(hardwareMap.get(DcMotor.class, LEFT_MOTOR), hardwareMap.get(DcMotor.class, RIGHT_MOTOR));
        myIntake = new Intake(hardwareMap.get(DcMotor.class, INTAKE_MOTOR));
        myShooter = new Shooter(hardwareMap.get(DcMotor.class, SHOOTER_MOTOR), hardwareMap.voltageSensor.iterator().next());
        myViseur = new Viseur(hardwareMap.get(Servo.class, VISEUR_SERVO));
        myFeeder = new Feeder(hardwareMap.get(Servo.class, FEEDER_SERVO));
        myPather = new Pather(hardwareMap.get(CRServo.class, PATHER_SERVO));

        // Boutons
        btnA = new ButtonReader(() -> gamepad1.a);
        btnB = new ButtonReader(() -> gamepad1.b);
        btnX = new ButtonReader(() -> gamepad1.x);
        btnL_Bumper = new ButtonReader(() -> gamepad1.left_bumper);
        btnL_Trigger = new ButtonReader(() -> gamepad1.left_trigger > 0.5);
        btnR_Bumper = new ButtonReader(() -> gamepad1.right_bumper);
        btnR_Trigger = new ButtonReader(() -> gamepad1.right_trigger > 0.5);
    }

    @Override
    public void runOpMode() {
        initialize();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. COMMANDES (INTENTIONS)

            // Shooter & Viseur (A, B, X)
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

            // Pather (RT)
            if (btnR_Trigger.wasJustPressed()) {
                myPather.toggleForward();
            }

            // Feeder (RB)
            if (btnR_Bumper.wasJustPressed()) {
                myFeeder.autoFire();
            }

            // Intake (LB / LT) : Actif pendant l'appui (Mode "Hold")
            if (btnL_Bumper.isDown()) {
                myIntake.collect();
            } else if (btnL_Trigger.isDown()) {
                myIntake.eject();
            } else {
                myIntake.stop();
            }

            // Pilotage (Sticks)
            myDriver.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

            // 2. ACTIONS (RÉALITÉ)
            myDriver.update();
            myIntake.update();
            myShooter.update();
            myViseur.update();
            myFeeder.update();
            myPather.update();

            // 3. TÉLÉMÉTRIE
            telemetry.addLine("--- MOUVEMENT ---");
            telemetry.addData("Châssis", myDriver.getState());
            telemetry.addData("Pather", myPather.getState() + " (" + myPather.getTargetState() + ")");

            telemetry.addLine("--- COLLECTE ---");
            telemetry.addData("Intake", myIntake.getState());

            telemetry.addLine("--- TIR ---");
            telemetry.addData("Shooter", myShooter.getState() + (myShooter.isReady() ? " [PRÊT]" : " [LOADING]"));
            telemetry.addData("Viseur", myViseur.getState() + " (" + myViseur.getTargetState() + ")");
            telemetry.addData("Feeder", myFeeder.getState() + " (" + myFeeder.getTargetState() + ")");

            telemetry.update();
        }
    }
}
