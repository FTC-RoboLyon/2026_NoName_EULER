package org.firstinspires.ftc.teamcode.euler.steps;

import org.firstinspires.ftc.teamcode.euler.Robot;
import org.firstinspires.ftc.teamcode.euler.Step;
import org.firstinspires.ftc.teamcode.euler.feeder.FeederState;
import org.firstinspires.ftc.teamcode.euler.viseur.ViseurState;

/**
 * Etape qui permet de
 * - règler le viseur
 * - mettre le shooter à la bonne vitesse
 * - puis quand c'est pret, actionne le Feeder (autoFire)
 */
public class Shoot implements Step {

    public enum ShootPosition {
        NEAR, FAR, MIDDLE
    }

    private enum InternalState {
        PREPARATION, READY_TO_SHOOT
    }

    private final ShootPosition position;
    private final int numberOfBalls;

    private boolean initialized;
    private boolean finished;
    private InternalState internalState;
    private int nbBallShooted;

    public Shoot(ShootPosition position, int numberOfBalls) {
        this.position = position;
        this.numberOfBalls = numberOfBalls;
    }

    @Override
    public void init(Robot robot) {
        switch (position) {
            case NEAR: {
                robot.getShooter().shootNear();
                robot.getViseur().aimNear();
            }
            case MIDDLE: {
                robot.getShooter().shootMiddle();
                robot.getViseur().aimMiddle();
            }
            case FAR: {
                robot.getShooter().shootFar();
                robot.getViseur().aimFar();
            }
        }
        this.internalState = InternalState.PREPARATION;
        this.nbBallShooted = 0;
        this.initialized = true;
        this.finished = false;
    }

    @Override
    public void run(Robot robot) {
        robot.update();

        if (robot.getViseur().getState() == ViseurState.IDLE && robot.getShooter().isReady()) {
            // viseur pret ET shooter pret
            this.internalState = InternalState.READY_TO_SHOOT;
        }

        if (this.internalState == InternalState.READY_TO_SHOOT && robot.getFeeder().getState() == FeederState.IDLE) {
            // internalState=SHOOTING ET feederState=IDLE
            if (this.nbBallShooted > this.numberOfBalls) {
                this.finished = true;
            } else {
                this.nbBallShooted++;
                robot.getFeeder().autoFire();
                robot.update();
            }
        }
    }

    @Override
    public void finish(Robot robot) {
        robot.getShooter().stop();
        robot.update();
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

