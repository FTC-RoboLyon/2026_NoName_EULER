/*******************************************************************************
 *
 * File        : Pose2d.java (v1.1)
 * Library     : LyonLib - FTC edition
 * Description : Minimal field pose: position in meters + heading in radians.
 *
 * Authors     : AKA (06/2026), last update by AKA (06/2026)
 *                                and inspired by WPILib's Pose2d
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 *
 *******************************************************************************/
package org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics;

/**
 * Minimal field pose: position in meters + heading in radians. Pure Java.
 *
 * <p>Convention matches WPILib: +x forward on the field, +y left, heading CCW.
 */
public class Pose2d
{
    public double xMeters;
    public double yMeters;
    public double headingRadians;

    public Pose2d()
    {
        this(0.0, 0.0, 0.0);
    }

    public Pose2d(double xMeters, double yMeters, double headingRadians)
    {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.headingRadians = headingRadians;
    }

    /**
     * Advances this pose by a robot-relative twist (dx, dy forward/left in
     * meters, dTheta in radians) using the exact pose exponential. This handles
     * curved motion correctly instead of assuming straight steps, so heading and
     * translation stay consistent even while turning.
     *
     * @return the new pose (this object is not modified).
     */
    public Pose2d exp(double dxMeters, double dyMeters, double dThetaRadians)
    {
        double sinTheta = Math.sin(dThetaRadians);
        double cosTheta = Math.cos(dThetaRadians);

        double s;
        double c;
        if (Math.abs(dThetaRadians) < 1e-9)
        {
            // Taylor expansion near zero to avoid dividing by ~0.
            s = 1.0 - dThetaRadians * dThetaRadians / 6.0;
            c = 0.5 * dThetaRadians;
        }
        else
        {
            s = sinTheta / dThetaRadians;
            c = (1.0 - cosTheta) / dThetaRadians;
        }

        // Displacement expressed in the robot's local frame.
        double dxLocal = dxMeters * s - dyMeters * c;
        double dyLocal = dxMeters * c + dyMeters * s;

        // Rotate the local displacement into the field frame by the current heading.
        double cosH = Math.cos(headingRadians);
        double sinH = Math.sin(headingRadians);

        return new Pose2d(
                xMeters + dxLocal * cosH - dyLocal * sinH,
                yMeters + dxLocal * sinH + dyLocal * cosH,
                headingRadians + dThetaRadians);
    }

    /** Returns an independent copy of this pose. */
    public Pose2d copy()
    {
        return new Pose2d(xMeters, yMeters, headingRadians);
    }

    /** [x, y, headingRadians] — the array layout field views expect for a pose. */
    public double[] toArray()
    {
        return new double[] {xMeters, yMeters, headingRadians};
    }

    @Override
    public String toString()
    {
        return String.format(
                "Pose2d(x=%.3f m, y=%.3f m, heading=%.1f deg)",
                xMeters, yMeters, Math.toDegrees(headingRadians));
    }
}