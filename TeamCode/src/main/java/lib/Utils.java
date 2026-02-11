package lib;

public class Utils {
    public static boolean IsInRange(double value, double target, double tolerance)
    {
        return (value >= target-tolerance && value <= target+tolerance);
    }
}
