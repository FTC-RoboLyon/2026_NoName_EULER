/*******************************************************************************
 * 
 * File        : NoOpLogger.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : A logger that discards everything. Used as the default 
 *               when no backend is provided.
 * 
 * Authors     : AKA (06/2026), last update by AKA (06/2026) 
 *                                
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.logging;

/**
 * A logger that discards everything. Used as the default when no backend is
 * provided.
 */
public class NoOpLogger implements RealtimeLogger 
{
    @Override
    public void log(String key, double value) {}

    @Override
    public void log(String key, boolean value) {}

    @Override
    public void log(String key, double[] values) {}

    @Override
    public void log(String key, String value) {}
}
