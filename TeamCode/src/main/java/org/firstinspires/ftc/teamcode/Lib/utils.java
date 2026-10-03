package org.firstinspires.ftc.teamcode.Lib;

public final class utils {
    public static boolean IsInRange(double value, double target, double tolerance)
    {
        return (value >= target-tolerance && value <= target+tolerance);
    }

    public static double getVoltageCompensated (double power, double voltage, double setpoint)
    //->Le seul pb de faire comme ca c'est que tu met pour tout tes subsystems une compensation pour 11V mais des fois on voudra mettre des trucs differents
    // genre l'intake a pas besoin de bcp de puissance on met à 9V alors que le shooter à 11V (ces valeurs que je viens de te donner sont aléatoires juste pour l'exemple)
    {
        double output = (power*voltage)/setpoint;

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

    public static double clamp(double value, double max, double min){
        return Math.min(Math.max(value, min), max);
    }
}
