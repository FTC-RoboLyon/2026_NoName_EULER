/*******************************************************************************
 * 
 * File        : ChassisSpeeds.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Robot-relative chassis speeds. Pure Java, no dependency.
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
 * Robot-relative chassis speeds. Pure Java, no WPILib dependency, so this type
 * is reused unchanged when porting to FTC.
 *
 * <p>Convention (matches WPILib so the mental model carries over):
 * <ul>
 *   <li>+vx    = forward</li>
 *   <li>+vy    = left</li>
 *   <li>+omega = counter-clockwise (CCW)</li>
 * </ul>
 */
public class ChassisSpeeds 
{
    public double vx; //in meters per second
    public double vy; //in metters per second
    public double omega; //in radians per second

    public ChassisSpeeds() 
    {
        this(0.0, 0.0, 0.0);
    }

    
    /**
     * Constructs a ChassisSpeeds instance representing the desired chassis velocity.
     *
     * <p>vx and vy are the linear velocities expressed in the robot's coordinate frame:
     * vx is the forward velocity (positive forward) and vy is the lateral velocity
     * (positive to the left). omega is the angular velocity about the Z-axis (positive counter-clockwise).
     *
     *
     * @param vx longitudinal (forward) velocity in m/s
     * @param vy lateral (sideways) velocity in m/s
     * @param omega angular velocity about the vertical axis in rad/s
     */
    public ChassisSpeeds(double vx, double vy, double omega) 
    {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    @Override
    public String toString() 
    {
        return String.format(
                "ChassisSpeeds(vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s)",
                vx, vy, omega);
    }
}
