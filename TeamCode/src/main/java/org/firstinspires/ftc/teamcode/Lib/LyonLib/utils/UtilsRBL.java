/*******************************************************************************
 * File         : UtilsRBL.java (v2.3)
 * Library      : LyonLib - FTC edition
 * Description  : mathematical and bit manipulation utilities
 * Authors      : AKA (2025), last update by AKA (06/2026)
 * Organization : Robo'Lyon - FRC Team 5553
 *                Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.utils;

import org.firstinspires.ftc.teamcode.Lib.LyonLib.logging.DebugUtils;

public final class UtilsRBL {

    private UtilsRBL() {}

    /* =============================================================================
     * METHODS : CONVERSIONS
     * ========================================================================== */

    /**
     * Converts revolutions per minute (RPM) to revolutions per second (RPS).
     *
     * @param rpm the value in RPM to be converted
     * @return the converted value in RPS
     */
    public static double RPMtoRPS(double rpm) {
        return rpm / 60.0;
    }

    /**
     * Converts revolutions per second (RPS) to revolutions per minute (RPM).
     *
     * @param rps the value in RPS to be converted
     * @return the converted value in RPM
     */
    public static double RPStoRPM(double rps) {
        return rps * 60.0;
    }

    /**
     * Normalize an angle in radians to the interval [0, 2π[.
     *
     * <p>Returns an equivalent angle to the provided value a such that the result lies
     * in the half-open interval [0, 2 * Math.PI[. Angles less than 0 or greater than
     * or equal to 2π are shifted by integer multiples of 2π until they fall within
     * this range.
     *
     * @param a the input angle in radians
     * @return an angle in radians equivalent to {@code a} and in the range [0, 2π[
     * @see #inputModulus(double, double, double)
     */
    public static double wrap0To2Pi(double a)    { return inputModulus(a, 0.0, 2*Math.PI); }

    /**
     * Normalize an angle in degrees to the interval [0, 360[.
     *
     * <p>Returns an equivalent angle to the provided value a such that the result lies
     * in the half-open interval [0, 360[. Angles less than 0 or greater than
     * or equal to 360 are shifted by integer multiples of 360 until they fall within
     * this range.
     *
     * @param a the input angle in degrees
     * @return an angle in degrees equivalent to {@code a} and in the range [0, 360[
     * @see #inputModulus(double, double, double)
     */
    public static double wrap0To360(double a)    { return inputModulus(a, 0.0, 360.0); }

    /**
     * Normalize an angle in radians to the interval [-π, π[.
     *
     * <p>Returns an equivalent angle to the provided value a such that the result lies
     * in the half-open interval [-π, π[. Angles less than -π or greater than
     * or equal to π are shifted by integer multiples of 2π until they fall within
     * this range.
     *
     * @param a the input angle in radians
     * @return an angle in radians equivalent to {@code a} and in the range [-π, π[
     * @see #inputModulus(double, double, double)
     */
    public static double wrapNegPiToPi(double a)  { return inputModulus(a, -Math.PI, Math.PI); }

    /* =============================================================================
     * METHODS : MATH UTILITIES
     * ========================================================================== */

    /**
     * Applies a deadband to the input value: values whose absolute magnitude is
     * less than the specified threshold are treated as zero; other values are
     * returned unchanged.
     *
     * @param a the input value
     * @param threshold the deadband threshold (non-negative)
     * @return zero if |a| < threshold, otherwise the original value a
     */
    public static double deadband(double a, double threshold) {
        return (Math.abs(a) < threshold) ? 0.0 : a;
    }

    /**
     * Clamps a value to the inclusive range [min, max].
     *
     * If max < min the behavior is unspecified; callers should ensure min <= max.
     *
     * @param min the lower bound
     * @param val the value to clamp
     * @param max the upper bound
     * @return val constrained to the range [min, max]
     */
    public static double clamp(double min, double val, double max) {
        DebugUtils.assertion(min <= max, "UtilsRBL.clamp: min must be less than or equal to max.");
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Clamps a value to the inclusive range [val - magnitude, val + magnitude].
     *
     * If magnitude < 0 the behavior is unspecified; callers should ensure magnitude >= 0.
     *
     * @param val the value to clamp
     * @param magnitude the maximum allowable deviation from val
     * @return val constrained to the range [val - magnitude, val + magnitude]
     */
    public static double clamp(double val, double magnitude) {
        DebugUtils.assertion(magnitude >= 0, "UtilsRBL.clamp: magnitude must be non-negative.");
        return clamp(-magnitude, val, magnitude);
    }

    /**
     * Tests whether a value lies within a symmetric tolerance of a target.
     *
     * @param val the value to test
     * @param target the target value
     * @param tolerance non-negative allowable deviation from target
     * @return true if |val - target| <= tolerance, otherwise false
     */
    public static boolean isInRange(double val, double target, double tolerance) {
        return Math.abs(val - target) <= tolerance;
    }

    /**
     * Performs linear interpolation between two values.
     *
     * @param a the start value (t == 0 yields a)
     * @param b the end value (t == 1 yields b)
     * @param t interpolation parameter (typically in [0, 1])
     * @return the interpolated value a + (b - a) * t
     */
    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double inputModulus(double input, double min, double max) {
        double range = max - min;
        DebugUtils.assertion(range > 0.0, "UtilsRBL.inputModulus: range must be positive (max > min).");
        if (range <= 0.0) return input;   // prevent division by zero or negative range in comp
        return ((input - min) % range + range) % range + min;
    }

    /**
     * Deadband with rescaling: values inside [-deadband, deadband] return 0, and
     * values outside are rescaled so the output is continuous (it ramps from 0
     * just past the deadband edge instead of jumping).
     */
    public static double applyDeadband(double value, double deadband)
    {
        if (Math.abs(value) <= deadband)
            return 0.0;
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    /** Squares the magnitude while keeping the sign (finer control near zero). */
    public static double squareKeepSign(double value)
    {
        return Math.copySign(value * value, value);
    }

    /* =============================================================================
     * METHODS : BIT MANIPULATION
     * ========================================================================== */

    public static int bitSet(int val, int bitId) {
        return val | (1 << bitId);
    }

    public static int bitClear(int val, int bitId) {
        return val & ~(1 << bitId);
    }

    public static int bitGet(int val, int bitId) {
        return (val >> bitId) & 1;
    }

    public static int bitToggle(int val, int bitId) {
        return val ^ (1 << bitId);
    }
}