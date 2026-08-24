package org.firstinspires.ftc.teamcode.Lib;

public final class utils {
    public static boolean IsInRange(double value, double target, double tolerance)// good
    {
        return (value >= target-tolerance && value <= target+tolerance);
    }
}
