package org.firstinspires.ftc.teamcode.euler.steps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;

/**
 * Step permettant de faire avancer  (ou reculer) le robot durant un temps définis
 */
public class ForwardByTime implements Step {

    private final int durationInMs;
    private final boolean forward;

    private ElapsedTime timer;
    private boolean initialized;
    private boolean finished;

    public ForwardByTime(int durationInMs, boolean forward) {
        this.durationInMs = durationInMs;
        this.forward = forward;
    }

    @Override
    public void init(Robot robot) {
        this.timer = new ElapsedTime();
        this.initialized = true;
        this.finished = false;
    }

    @Override
    public void run(Robot robot) {
        if (timer.milliseconds() < durationInMs) {
            // faire avancer (ou reculer) le robot
            double power = 0.5;
            if (!forward) {
                power = -power; // recule
            }
            robot.getDriver().drive(power, power);
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
