package org.firstinspires.ftc.teamcode.euler.steps;

import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;
import org.firstinspires.ftc.teamcode.euler.intake.IntakeState;

public class StopCollect implements Step {

    private boolean initialized;
    private boolean finished;

    @Override
    public void init(Robot robot) {
        this.initialized = true;
        this.finished = false;
    }

    @Override
    public void run(Robot robot) {
        robot.getIntake().stop();
        robot.getIntake().update();
        if (robot.getIntake().getState() == IntakeState.IDLE) {
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
