/*******************************************************************************
 *
 * File        : DebugUtils.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Debugging utilities for logging and assertions.
 *
 * Authors     : AKA (2026), last update by AKA (2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 *
 * Usage       : Set DEBUG_MODE = true to enable debug mode.
 *               NEVER set to true in competition mode.
 *               Because DEBUG_MODE is a static final boolean, the Java compiler
 *               will eliminate dead branches at compile time, just like 
 *               #ifdef in c++.
 *
 *******************************************************************************/
 
package org.firstinspires.ftc.teamcode.Lib.LyonLib.logging;

import com.qualcomm.robotcore.util.RobotLog;

public final class DebugUtils
{
    private DebugUtils() {}

    /**
     * Tag used for all RobotLog output, so it's easy to filter in logcat.
     */
    private static final String TAG = "LyonLib";
    private static final boolean DEBUG_MODE = true; //change this constant to false in comp

    /**
     * In DEBUG mode  : throws an AssertionError if the condition is false.
     * In NORMAL mode : simply logs the assertion message.
     *
     * @param cond Condition to verify.
     * @param msg  Message logged / thrown if the condition is false.
     * @return the result of the assertion
     */
    public static boolean assertion(boolean cond, String msg) {
        if (DEBUG_MODE) {
            if (!cond) {
                throw new AssertionError("[ASSERT] " + msg);
            }
        } else {
            // In normal mode, we don't throw, but we still log the failure.
            if (!cond) {
                RobotLog.ee(TAG, "[ASSERT] %s", msg);
                return false;
            }
        }
        return true;
    }
    //TODO : test assert in comp

    /**
     * Logs the message only in DEBUG mode, otherwise no-op.
     *
     * @param msg Message to display.
     */
    public static void debugLog(String msg) {
        if (DEBUG_MODE) {
            RobotLog.dd(TAG, "[DEBUG] %s", msg);
        }
    }

    /**
     * Reports an error via the FTC RobotLog system.
     *
     * @param msg Message of the error.
     */
    public static void errorLog(String msg) {
        RobotLog.ee(TAG, "[ERROR] %s", msg);
    }

    /**
     * Reports a warning via the FTC RobotLog system.
     *
     * @param msg Message of the warning.
     */
    public static void warningLog(String msg) {
        RobotLog.ww(TAG, "[WARNING] %s", msg);
    }

    /**
     * Logs an info message, always active.
     *
     * @param msg Info message.
     */
    public static void infoLog(String msg) {
        RobotLog.ii(TAG, "[INFO] %s", msg);
    }
}

