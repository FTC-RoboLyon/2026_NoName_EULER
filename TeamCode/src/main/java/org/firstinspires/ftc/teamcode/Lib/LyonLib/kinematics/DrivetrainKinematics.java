/*******************************************************************************
 *
 * File        : DrivetrainKinematics.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Hardware-agnostic AND drivetrain-agnostic abstraction
 *               for the drive subsystem. The subsystem talks only to
 *               this interface.
 *
 * Authors     : VTT (07/2026), last update by VTT (07/2026)
 *                                and inspired by WPILib
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 *
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.kinematics;

public interface DrivetrainKinematics<WheelSpeeds> {
    /** Inverse kinematics: desired chassis motion -&gt; required wheel speeds. */
    public WheelSpeeds toWheelSpeeds(ChassisSpeeds speeds);

    /** Forward kinematics: measured wheel speeds -&gt; resulting chassis motion. */
    public ChassisSpeeds toChassisSpeeds(WheelSpeeds w);
}
