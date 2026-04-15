package org.firstinspires.ftc.teamcode.euler.driver;

public enum RunMode {
    NORMAL(1),
    PARK(0.5);

    private double coef;

    RunMode(double coef) {
        this.coef = coef;
    }

    public double getCoef() {
        return coef;
    }
}
