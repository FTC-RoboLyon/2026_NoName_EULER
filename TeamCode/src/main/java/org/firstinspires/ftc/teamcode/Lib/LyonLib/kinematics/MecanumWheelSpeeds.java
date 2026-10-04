/*******************************************************************************
 * 
 * File        : MecanumWheelSpeeds.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Wheel speeds for a 4-wheel mecanum drive, in m/s. Pure Java.
 * 
 * Authors     : AKA (06/2026), last update by AKA (06/2026) 
 *                                and inspired by Team 2910
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
package org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics;

/**
 * Wheel speeds for a 4-wheel mecanum drive, in meters/second. Pure Java.
 */
public class MecanumWheelSpeeds 
{
    public double frontLeft; //in meters per second
    public double frontRight; //in meters per second
    public double rearLeft; // in meters per second
    public double rearRight; // in meters per second

    public MecanumWheelSpeeds() {}

    public MecanumWheelSpeeds(double fl, double fr, double rl, double rr) 
    {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.rearLeft = rl;
        this.rearRight = rr;
    }

    /**
     * Scales every wheel speed down proportionally if any of them exceeds
     * {@code maxSpeed}. This preserves the commanded direction instead of
     * clipping individual wheels (which would distort the motion).
     */
    public void desaturate(double maxSpeed) 
    {
        double highest = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(rearLeft), Math.abs(rearRight)));

        if (highest > maxSpeed && highest > 1e-9) 
        {
            double scale = maxSpeed / highest;
            frontLeft *= scale;
            frontRight *= scale;
            rearLeft *= scale;
            rearRight *= scale;
        }
    }
}
