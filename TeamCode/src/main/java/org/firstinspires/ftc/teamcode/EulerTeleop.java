package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.utils.ButtonReader;

/**
 * EulerTeleop - OpMode principal.
 * Version itérative (OpMode) utilisant la classe Robot.
 */
@TeleOp(name = "EulerTeleop", group = "Euler")
public class EulerTeleop extends OpMode {
    private Robot robot;

    private ButtonReader btnA;
    private ButtonReader btnY;
    private ButtonReader btnB;
    private ButtonReader btnX;
    private ButtonReader btnL_Bumper;
    private ButtonReader btnL_Trigger;
    private ButtonReader btnR_Bumper;
    private ButtonReader btnR_Trigger;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        // Configuration des boutons
        btnA = new ButtonReader(() -> gamepad1.a);
        btnB = new ButtonReader(() -> gamepad1.b);
        btnX = new ButtonReader(() -> gamepad1.x);
        btnL_Bumper = new ButtonReader(() -> gamepad1.left_bumper);
        btnL_Trigger = new ButtonReader(() -> gamepad1.left_trigger > 0.5);
        btnR_Bumper = new ButtonReader(() -> gamepad1.right_bumper);
        btnR_Trigger = new ButtonReader(() -> gamepad1.right_trigger > 0.5);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. COMMANDES (INTENTIONS)
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

        if (btnR_Trigger.wasJustPressed()) robot.getPather().toggleBackward();
        if (btnR_Bumper.wasJustPressed()) robot.getFeeder().autoFire();
        if (btnY.wasJustPressed()) robot.getDriver().toggleParkMode();

        if (btnL_Bumper.isDown()) {
            robot.getIntake().collect();
        } else if (btnL_Trigger.isDown()) {
            robot.getIntake().eject();
        } else {
            robot.getIntake().stop();
        }

        robot.getDriver().drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        // 2. MISE À JOUR (ACTIONS)
        robot.update();

        // 3. TÉLÉMÉTRIE
        displayTelemetry();
    }

    private void displayTelemetry() {
        robot.getTelemetries()
                .forEach(robotTelemetry -> {
                    telemetry.addData(robotTelemetry.getCaption(), robotTelemetry.getValue());
                });

        telemetry.update();
    }
}
