package org.firstinspires.ftc.teamcode.euler.steps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;

/**
 * Step permettant de faire pivoter le robot d'un angle donné
 */
public class Rotate implements Step {

    private final int angleInDegree;

    private double targetAngle;
    private boolean initialized;
    private boolean finished;

    public Rotate(int angleInDegree) {
        this.angleInDegree = angleInDegree;
    }

    @Override
    public void init(Robot robot) {
        // calcul de l'angle à atteindre en fonction de là ou est le robot au moment de l'init
        this.targetAngle = angleInDegree + robot.getCompass().getHeading(AngleUnit.DEGREES);
        this.initialized = true;
        this.finished = false;
    }

    @Override
    public void run(Robot robot) {
        double actualHeading = robot.getCompass().getHeading(AngleUnit.DEGREES);

        if (actualHeading > targetAngle) {
            // faire pivoter le robot
            robot.getDriver().drive(0.5, -0.5);
            robot.getDriver().update();
        } else {
            this.finished = true;
        }
    }


    @Override
    public void finish(Robot robot) {
        // arreter le robot
        robot.getDriver().drive(0, 0);
        robot.getDriver().update();
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
