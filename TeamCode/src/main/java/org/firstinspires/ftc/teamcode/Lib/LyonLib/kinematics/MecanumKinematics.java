/*******************************************************************************
 * 
 * File        : MecanumKinematics.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Hardware-agnostic AND drivetrain-agnostic abstraction 
 *               for the drive subsystem. The subsystem talks only to 
 *               this interface.
 * 
 * Authors     : AKA (06/2026), last update by VTT (07/2026)
 *                                and inspired by Team 2910
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics;

/**
 * Mecanum kinematics: converts between chassis speeds and individual wheel
 * speeds.
 *
 * <p>Assumes a symmetric layout (geometric center == center of rotation):
 * <pre>
 *   front-left  at (+wheelBase/2, +trackWidth/2)
 *   front-right at (+wheelBase/2, -trackWidth/2)
 *   rear-left   at (-wheelBase/2, +trackWidth/2)
 *   rear-right  at (-wheelBase/2, -trackWidth/2)
 * </pre>
 * Standard rollers-at-45-degrees configuration. All math is in SI units; the
 * hardware layer converts m/s &lt;-&gt; duty-cycle.
 */
public class MecanumKinematics implements DrivetrainKinematics<MecanumWheelSpeeds>
{
    /** Lever arm from center to a wheel in meters : (trackWidth/2 + wheelBase/2) (in meters)*/
    private final double lever;


    /**
     * Creates a MecanumKinematics configured for a robot with the specified track-width and wheelbase.
     *
     * <p>The constructor precomputes the lever arm used by the kinematics to relate chassis
     * angular velocity to individual wheel linear velocities. The lever is computed as:
     *
     * <pre>
     * lever = (trackwidth / 2.0) + (wheelBase / 2.0)
     * </pre>
     *
     * This lever represents the effective perpendicular distance from the robot center to a
     * wheel along the diagonal direction used in rotational contributions.
     *
     * @param trackwidth : the lateral distance between left and right wheel centers, in meters
     * @param wheelBase : the longitudinal distance between front and back wheel centers, in meters
     *
     * @implNote :
     * Both dimensions should be measured between wheel centers and supplied in meters ! The
     * resulting lever is expressed in meters and is used internally for converting angular
     * velocity (rad/s) to wheel linear velocity (m/s) contributions.
     */
    public MecanumKinematics(double trackwidth, double wheelBase)
    {
        this.lever = (trackwidth / 2.0) + (wheelBase / 2.0);
    }

    /** Inverse kinematics: desired chassis motion -&gt; required wheel speeds. */
    @Override
    public MecanumWheelSpeeds toWheelSpeeds(ChassisSpeeds speeds) 
    {
        double vx = speeds.vx;
        double vy = speeds.vy;
        double w = speeds.omega * lever;

        return new MecanumWheelSpeeds(
                vx - vy - w, // front-left
                vx + vy + w, // front-right
                vx + vy - w, // rear-left
                vx - vy + w); // rear-right
    }

    /** Forward kinematics: measured wheel speeds -&gt; resulting chassis motion. */
    @Override
    public ChassisSpeeds toChassisSpeeds(MecanumWheelSpeeds w)
    {
        double fl = w.frontLeft;
        double fr = w.frontRight;
        double rl = w.rearLeft;
        double rr = w.rearRight;

        double vx = (fl + fr + rl + rr) / 4.0;
        double vy = (-fl + fr + rl - rr) / 4.0;
        double omega = (-fl + fr - rl + rr) / (4.0 * lever);

        return new ChassisSpeeds(vx, vy, omega);
    }
}
