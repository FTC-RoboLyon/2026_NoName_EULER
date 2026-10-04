/*******************************************************************************
 *
 * File        : CsvFileLogger.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : writes a plain text CSV file — one row per cycle, one column
 *               per logged key. Opens directly in any spreadsheet. Pure Java
 *               (java.io only), so it works in FRC and FTC alike; only
 *               the file path differs.
 *
 * Authors     : AKA (06/2026), last update by AKA (06/2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 *
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.logging;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * {@link RealtimeLogger} backend that writes a plain text CSV file — one row per
 * cycle, one column per logged key. Opens directly in any spreadsheet. Pure Java
 * (java.io only), so it works in FRC and FTC alike; only the file path differs.
 *
 * <p>How it works: values logged during a cycle are accumulated, then
 * {@link #flush()} commits one row. Call flush() once per loop — the drive
 * subsystem already does this at the end of periodic().
 *
 * <p>The column set is locked from the first committed row, so the keys logged
 * each cycle should be consistent (which {@code DriveTelemetry} guarantees). A
 * {@code double[]} value is expanded into one column per element: {@code key[0]},
 * {@code key[1]}, ... (e.g. Pose -> Pose[0], Pose[1], Pose[2]).
 */
public class CsvFileLogger implements RealtimeLogger {
    private final BufferedWriter writer;
    private final long startNanos = System.nanoTime();

    private final Map<String, String> currentRow = new LinkedHashMap<>();
    private List<String> header = null;
    private boolean headerWritten = false;

    public CsvFileLogger(String filePath) {
        try {
            writer = new BufferedWriter(new FileWriter(filePath));
        } catch (IOException e) {
            throw new UncheckedIOException("Could not open CSV log: " + filePath, e);
        }
    }

    /**
     * Convenience factory: builds a timestamped file
     * {@code {directory}/{prefix}_yyyyMMdd_HHmmss.csv}.
     */
    public static CsvFileLogger inDirectory(String directory, String prefix) {
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        String sep = directory.endsWith("/") ? "" : "/";
        return new CsvFileLogger(directory + sep + prefix + "_" + stamp + ".csv");
    }

    @Override
    public void log(String key, double value) {
        currentRow.put(key, Double.toString(value));
    }

    @Override
    public void log(String key, boolean value) {
        currentRow.put(key, value ? "1" : "0");
    }

    @Override
    public void log(String key, String value) {
        currentRow.put(key, escape(value));
    }

    @Override
    public void log(String key, double[] values) {
        for (int i = 0; i < values.length; i++) {
            currentRow.put(key + "[" + i + "]", Double.toString(values[i]));
        }
    }

    @Override
    public void flush() {
        if (currentRow.isEmpty()) {
            return;
        }
        try {
            if (!headerWritten) {
                header = new ArrayList<>(currentRow.keySet());
                writer.write("Timestamp");
                for (String column : header) {
                    writer.write("," + column);
                }
                writer.newLine();
                headerWritten = true;
            }

            double elapsed = (System.nanoTime() - startNanos) / 1e9;
            writer.write(Double.toString(elapsed));
            for (String column : header) {
                writer.write("," + currentRow.getOrDefault(column, ""));
            }
            writer.newLine();

            // Persist every row: a robot is usually powered off, never closed.
            writer.flush();
        } catch (IOException e) {
            throw new UncheckedIOException("CSV log write failed", e);
        }
        currentRow.clear();
    }

    /** Closes the file. Optional, since every row is already flushed to disk. */
    public void close() {
        try {
            writer.close();
        } catch (IOException ignored) {
            // best effort
        }
    }

    private static String escape(String value) {
        if (value == null) {
            return "";
        }
        if (value.contains(",") || value.contains("\"") || value.contains("\n")) {
            return "\"" + value.replace("\"", "\"\"") + "\"";
        }
        return value;
    }
}
