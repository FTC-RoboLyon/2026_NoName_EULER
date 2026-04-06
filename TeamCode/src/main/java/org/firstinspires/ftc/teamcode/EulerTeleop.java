package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.utils.ButtonReader;

/**
 * EulerTeleop - OpMode principal.
 * Utilise la classe Robot pour simplifier la gestion des sous-systèmes.
 */
@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends LinearOpMode {
    private Robot robot;

    private ButtonReader btnA;
    private ButtonReader btnB;
    private ButtonReader btnX;
    private ButtonReader btnL_Bumper;
    private ButtonReader btnL_Trigger;
    private ButtonReader btnR_Bumper;
    private ButtonReader btnR_Trigger;

    void initialize() {
        robot = new Robot(hardwareMap);

        // Configuration des boutons
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

            // Shooter & Viseur
            if (btnA.wasJustPressed()) {
                robot.getShooter().toggleShootNear();
                robot.getViseur().aimNear();
            } else if (btnB.wasJustPressed()) {
                robot.getShooter().toggleShootMiddle();
                robot.getViseur().aimMiddle();
            } else if (btnX.wasJustPressed()) {
                robot.getShooter().toggleShootFar();
                robot.getViseur().aimFar();
            }

            // Pather & Feeder
            if (btnR_Trigger.wasJustPressed()) robot.getPather().toggleBackward();
            if (btnR_Bumper.wasJustPressed()) robot.getFeeder().autoFire();

            // Intake (Mode Hold)
            if (btnL_Bumper.isDown()) {
                robot.getIntake().collect();
            } else if (btnL_Trigger.isDown()) {
                robot.getIntake().eject();
            } else {
                robot.getIntake().stop();
            }

            // Châssis
            robot.getDriver().drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);


            // 2. MISE À JOUR (ACTIONS)
            robot.update();


            // 3. TÉLÉMÉTRIE
            displayTelemetry();
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("--- MOUVEMENT ---");
        telemetry.addData("Châssis", robot.getDriver().getState());
        telemetry.addData("Pather", robot.getPather().getState() + " (" + robot.getPather().getTargetState() + ")");

        telemetry.addLine("--- COLLECTE ---");
        telemetry.addData("Intake", robot.getIntake().getState());

        telemetry.addLine("--- TIR ---");
        telemetry.addData("Shooter", robot.getShooter().getState() + (robot.getShooter().isReady() ? " [PRÊT]" : " [LOADING]"));
        telemetry.addData("Viseur", robot.getViseur().getState() + " (" + robot.getViseur().getTargetState() + ")");
        telemetry.addData("Feeder", robot.getFeeder().getState() + " (" + robot.getFeeder().getTargetState() + ")");

        telemetry.update();
    }
}
