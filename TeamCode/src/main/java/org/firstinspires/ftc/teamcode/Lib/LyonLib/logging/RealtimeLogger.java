/*******************************************************************************
 * 
 * File        : RealtimeLogger.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Portable real-time logging seam. The logic layers call this 
 *               with plain (key, value) pairs; the chosen backend decides 
 *               how to emit them. 
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
 * Portable real-time logging seam. The logic layers call this with plain
 * (key, value) pairs; the chosen backend decides how to emit them.
 *
 * <p>FRC: {@code NetworkTablesLogger}. FTC : an FtcDashboard / Telemetry
 * implementation. Tests / logging-off: {@link NoOpLogger}.
 */
public interface RealtimeLogger 
{
    void log(String key, double value);

    void log(String key, boolean value);

    void log(String key, double[] values);

    void log(String key, String value);

    /** Optional: force-push buffered values. Most backends auto-publish. */
    default void flush() {}
}
