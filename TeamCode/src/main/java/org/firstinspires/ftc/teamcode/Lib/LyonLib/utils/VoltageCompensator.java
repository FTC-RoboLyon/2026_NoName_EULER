/*******************************************************************************
 *
 * File        : VoltageCompensator.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Battery voltage compensation for FTC robots.
 * Authors     : AKA (06/2026), last update by AKA (06/2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 *
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * Battery voltage compensation for FTC.
 *
 * <p>A duty-cycle command of, say, 0.5 produces less output as the battery sags
 * (13.5 V fresh -> ~11 V under load). This converts a desired motor VOLTAGE into
 * the duty cycle that actually delivers it at the present battery voltage, so
 * behaviour stays consistent as the pack drains.
 *
 * <p>The battery voltage is read from the hub's {@link VoltageSensor}, sampled at
 * a bounded rate and lightly EMA-filtered (battery voltage is noisy under load).
 * A bad/zero read falls back to the nominal voltage to avoid divide-by-zero.
 */
public class VoltageCompensator {
    private final List<VoltageSensor> sensors = new ArrayList<>();
    private final double nominalVoltage;
    private final long refreshPeriodNanos;
    private final double alpha;

    private double filteredVoltage;
    private long lastReadNanos = 0L;
    private boolean initialized = false;

    /** Defaults: 12 V nominal, resample every 50 ms, EMA alpha 0.3. */
    public VoltageCompensator(HardwareMap hardwareMap) {
        this(hardwareMap, 12.0, 0.05, 0.3);
    }

    /**
     * @param nominalVoltage       the voltage your commands are referenced to (e.g. 12.0)
     * @param refreshPeriodSeconds minimum time between sensor reads
     * @param filterAlpha          EMA smoothing in (0, 1]; higher = faster, less smooth
     */
    public VoltageCompensator(
            HardwareMap hardwareMap, double nominalVoltage, double refreshPeriodSeconds, double filterAlpha) {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            sensors.add(sensor);
        }
        this.nominalVoltage = nominalVoltage;
        this.refreshPeriodNanos = (long) (refreshPeriodSeconds * 1e9);
        this.alpha = filterAlpha;
        this.filteredVoltage = nominalVoltage; // sensible starting value
    }

    /** Smoothed battery voltage, refreshed at the configured rate. */
    public double getVoltage() {
        long now = System.nanoTime();
        if (!initialized || now - lastReadNanos >= refreshPeriodNanos) {
            double raw = readRaw();
            if (raw > 0.0) {
                filteredVoltage = initialized ? alpha * raw + (1.0 - alpha) * filteredVoltage : raw;
                initialized = true;
            }
            lastReadNanos = now;
        }
        return filteredVoltage;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    /** Duty cycle in [-1, 1] that applies ~{@code desiredVolts} at the motor now. */
    public double compensate(double desiredVolts) {
        double voltage = getVoltage();
        if (voltage < 1.0) {
            voltage = nominalVoltage; // guard against bad/zero reads
        }
        return clamp(desiredVolts / voltage);
    }

    /**
     * Scales a nominal-referenced power command in [-1, 1] so it delivers the same
     * effective voltage regardless of battery sag. Equivalent to treating the
     * power as a fraction of {@link #getNominalVoltage()}.
     */
    public double compensatePower(double power) {
        return compensate(power * nominalVoltage);
    }

    private double readRaw() {
        double max = 0.0;
        for (VoltageSensor sensor : sensors) {
            double v = sensor.getVoltage();
            if (v > max) {
                max = v; // the battery sensor reports the real value; others may read 0
            }
        }
        return max;
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
