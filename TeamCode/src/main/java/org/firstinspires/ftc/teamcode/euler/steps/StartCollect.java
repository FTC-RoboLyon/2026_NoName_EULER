package org.firstinspires.ftc.teamcode.euler.steps;

import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;
import org.firstinspires.ftc.teamcode.euler.intake.IntakeState;

/**
 * Met en marche l'intake
 */
public class StartCollect implements Step {

    private boolean initialized;
    private boolean finished;

    @Override
    public void init(Robot robot) {
        this.initialized = true;
        this.finished = false;
    }

    @Override
    public void run(Robot robot) {
        robot.getIntake().collect();
        robot.getIntake().update();
        if (robot.getIntake().getState() == IntakeState.COLLECT) {
            this.finished = true;
        }
    }

    @Override
    public void finish(Robot robot) {
        // rien
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
