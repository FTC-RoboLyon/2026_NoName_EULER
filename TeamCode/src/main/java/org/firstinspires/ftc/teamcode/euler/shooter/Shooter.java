package org.firstinspires.ftc.teamcode.euler.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter {

    final DcMotor shooter;

    public Shooter(DcMotor shooter) {
        this.shooter = shooter;
    }

    public void toggleShootNear() {
        if (getState() == ShooterState.IDLE) {
            shooter.setPower(0.3);
        } else {
            stop();
        }
    }

    public void toggleShootMiddle() {
        if (getState() == ShooterState.IDLE) {
            shooter.setPower(0.5);
        } else {
            stop();
        }
    }

    public void toggleShootFar() {
        if (getState() == ShooterState.IDLE) {
            shooter.setPower(0.8);
        } else {
            stop();
        }
    }

    private void stop() {
        shooter.setPower(0);
    }

    public ShooterState getState() {
        if (shooter.getPower() == 0) {
            return ShooterState.IDLE;
        } else {
            return ShooterState.SHOOTING;
        }
    }
}

