package org.firstinspires.ftc.teamcode.Lib;

public final class utils {
    public static boolean IsInRange(double value, double target, double tolerance)// good
    {
        return (value >= target-tolerance && value <= target+tolerance);
    }

    public static double getVoltageCompensated (double power, double voltage){
        double output = (power*voltage)/11;

        if (Math.abs(output) > 1)
            output /= Math.abs(output);

        return output;
    }


    /**
     * A function that take a tickPerSecond value and convert it to a RotationPerMinutes value
     * @param ticksPerSecond the value we want to convert, in number of tick per second
     * @param TPR the number of tick per rotation we use for the conversion
     * @param gearRatio the gearRatio of our motor, if we have one (else just set it to 1)
     * @return the converted value of tickPerSecond
     */
    public static double TickPerSecondToRPM(double ticksPerSecond, double TPR, double gearRatio){
        return ( (ticksPerSecond/TPR) * 60 ) / gearRatio;
    }
}
