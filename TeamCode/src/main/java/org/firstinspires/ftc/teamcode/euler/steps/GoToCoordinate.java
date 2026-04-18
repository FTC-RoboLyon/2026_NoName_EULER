package org.firstinspires.ftc.teamcode.euler.steps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;

public class GoToCoordinate implements Step {

    private final Pose2D target;

    private boolean initialized = false;
    private boolean finished = false;

    // Constantes de précision
    private static final double DISTANCE_TOLERANCE = 20.0; // mm
    private static final double HEADING_TOLERANCE = Math.toRadians(3.0); // 3 degrés

    // Gains de puissance (P-Controller)
    private static final double Kp_DRIVE = 0.05;
    private static final double Kp_TURN = 0.8;

    public GoToCoordinate(Pose2D target) {
        this.target = target;
    }

    @Override
    public void init(Robot robot) {
        this.initialized = true;
        this.finished = false;
    }

    @Override
    public void run(Robot robot) {
        // 1. Récupérer la position actuelle
        Pose2D currentPose = robot.getOdometry().getPose();
        double currentX = currentPose.getX(DistanceUnit.MM);
        double currentY = currentPose.getY(DistanceUnit.MM);
        double currentHeading = currentPose.getHeading(AngleUnit.RADIANS);

        // 2. Calculer la distance vers la cible
        double deltaX = target.getX(DistanceUnit.MM) - currentX;
        double deltaY = target.getY(DistanceUnit.MM) - currentY;
        double distanceToTarget = Math.hypot(deltaX, deltaY);

        // PHASE 1 : En route vers la coordonnée
        if (distanceToTarget > DISTANCE_TOLERANCE) {
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double headingError = wrapAngle(angleToTarget - currentHeading);

            double drivePower = Math.min(distanceToTarget * Kp_DRIVE, 0.7);
            double turnPower = headingError * Kp_TURN;

            double leftPower = drivePower - turnPower;
            double rightPower = drivePower + turnPower;

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            robot.getDriver().drive(leftPower, rightPower);
        }
        // PHASE 2 : Arrivé, on ajuste le cap final
        else {
            double finalHeadingError = wrapAngle(target.getHeading(AngleUnit.RADIANS) - currentHeading);

            if (Math.abs(finalHeadingError) < HEADING_TOLERANCE) {
                this.finished = true;
                robot.getDriver().drive(0, 0);
            } else {
                // Rotation sur place
                double turnPower = finalHeadingError * Kp_TURN;
                // Cap de sécurité pour la rotation finale
                turnPower = Math.max(-0.5, Math.min(turnPower, 0.5));
                robot.getDriver().drive(-turnPower, turnPower);
            }
        }

        robot.getDriver().update();
    }

    @Override
    public void finish(Robot robot) {
        robot.getDriver().drive(0, 0);
        robot.getDriver().update();
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
