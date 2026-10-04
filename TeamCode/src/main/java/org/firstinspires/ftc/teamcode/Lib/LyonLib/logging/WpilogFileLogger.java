/*******************************************************************************
 *
 * File        : WpilogFileLogger.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : writes a binary WPILOG (.wpilog) file directly, implementing
 *               the WPILib Data Log File Format v1.0. NO WPILib dependency,
 *               pure Java, so it runs on an FRC roboRIO and an FTC Control
 *               Hub alike. AdvantageScope opens the file natively: no CSV
 *               import step, correct types, and an array field
 *               (Pose = [x, y, theta]) is usable directly as a " Pose2d".
 *
 * Authors     : AKA (06/2026), last update by AKA (06/2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 *
 *******************************************************************************/

package org.firstinspires.ftc.teamcode.Lib.LyonLib.logging;
 
import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
 
/**
 * Writes a binary WPILOG (.wpilog) file directly, implementing the WPILib Data
 * Log File Format v1.0. NO WPILib dependency — pure Java, so it runs on an FRC
 * roboRIO and an FTC Control Hub alike. AdvantageScope opens the file natively:
 * no CSV import step, correct types, and an array field (Pose = [x, y, theta])
 * is usable directly as a Pose2d.
 *
 * <p>Records on change: a value is only written when it differs from the last
 * one for that key, so constant signals cost a single record. An optional
 * minimum period rate-limits the changing signals.
 *
 * <p>Values logged during a loop are committed by {@link #flush()} (which the
 * drive subsystem already calls once per loop), giving one timestamp per cycle.
 */
public class WpilogFileLogger implements RealtimeLogger {
    private final OutputStream out;
    private final long startNanos = System.nanoTime();
    private final long minPeriodNanos;
 
    private final Map<String, Object> pending = new LinkedHashMap<>();
    private final Map<String, Object> lastCommitted = new HashMap<>();
    private final Map<String, Integer> entryIds = new HashMap<>();
    private int nextEntryId = 1;
 
    private boolean emittedOnce = false;
    private long lastEmitNanos = 0L;
 
    public WpilogFileLogger(String filePath) {
        this(filePath, 0.0);
    }
 
    /** @param minPeriodSeconds minimum time between commits (0 = every loop). */
    public WpilogFileLogger(String filePath, double minPeriodSeconds) {
        this.minPeriodNanos = (long) (minPeriodSeconds * 1e9);
        try {
            File file = new File(filePath);
            File parent = file.getParentFile();
            if (parent != null) {
                parent.mkdirs();
            }
            out = new BufferedOutputStream(new FileOutputStream(file));
            writeFileHeader();
        } catch (IOException e) {
            throw new UncheckedIOException("Could not open WPILOG: " + filePath, e);
        }
    }
 
    /**
     * Creates a logger with an auto-generated, non-overwriting name:
     * {@code FTC_yyyyMMdd_HHmmss.wpilog}. A numeric suffix ({@code _1}, {@code _2},
     * ...) is added if that name already exists — which covers the case where the
     * robot clock isn't set yet at boot and several runs land in the same second.
     */
    public static WpilogFileLogger inDirectory(String directory) {
        return inDirectory(directory, "FTC");
    }
 
    /** Same, with a custom prefix instead of {@code FRC}. */
    public static WpilogFileLogger inDirectory(String directory, String prefix) {
        File dir = new File(directory);
        dir.mkdirs();
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        return new WpilogFileLogger(uniqueFile(dir, prefix + "_" + stamp).getPath());
    }
 
    /** Returns a .wpilog path in {@code directory} that does not exist yet. */
    private static File uniqueFile(File directory, String baseName) {
        File candidate = new File(directory, baseName + ".wpilog");
        for (int n = 1; candidate.exists(); n++) {
            candidate = new File(directory, baseName + "_" + n + ".wpilog");
        }
        return candidate;
    }
 
    @Override
    public void log(String key, double value) {
        pending.put(key, value);
    }
 
    @Override
    public void log(String key, boolean value) {
        pending.put(key, value);
    }
 
    @Override
    public void log(String key, String value) {
        pending.put(key, value);
    }
 
    @Override
    public void log(String key, double[] values) {
        pending.put(key, values.clone());
    }
 
    @Override
    public void flush() {
        if (pending.isEmpty()) {
            return;
        }
        long now = System.nanoTime();
        if (emittedOnce && (now - lastEmitNanos) < minPeriodNanos) {
            pending.clear();
            return;
        }
        emittedOnce = true;
        lastEmitNanos = now;
 
        long timestampMicros = (now - startNanos) / 1000L;
        try {
            for (Map.Entry<String, Object> e : pending.entrySet()) {
                commit(e.getKey(), e.getValue(), timestampMicros);
            }
            out.flush(); // persist each cycle: robots get powered off, not closed
        } catch (IOException ex) {
            throw new UncheckedIOException("WPILOG write failed", ex);
        }
        pending.clear();
    }
 
    public void close() {
        try {
            out.close();
        } catch (IOException ignored) {
            // best effort
        }
    }
 
    // ---- value handling -------------------------------------------------
 
    private void commit(String key, Object value, long timestampMicros) throws IOException {
        Integer id = entryIds.get(key);
        if (id == null) {
            id = nextEntryId++;
            entryIds.put(key, id);
            writeStartRecord(id, key, typeOf(value), timestampMicros);
        }
        if (unchanged(lastCommitted.get(key), value)) {
            return;
        }
        writeRecord(id, timestampMicros, encode(value));
        lastCommitted.put(key, value instanceof double[] ? ((double[]) value).clone() : value);
    }
 
    private static boolean unchanged(Object previous, Object current) {
        if (previous == null) {
            return false;
        }
        if (current instanceof double[] && previous instanceof double[]) {
            return Arrays.equals((double[]) previous, (double[]) current);
        }
        return previous.equals(current);
    }
 
    private static String typeOf(Object value) {
        if (value instanceof Double) {
            return "double";
        }
        if (value instanceof Boolean) {
            return "boolean";
        }
        if (value instanceof double[]) {
            return "double[]";
        }
        return "string";
    }
 
    private static byte[] encode(Object value) {
        if (value instanceof Double) {
            return leDouble((Double) value);
        }
        if (value instanceof Boolean) {
            return new byte[] {(byte) (((Boolean) value) ? 1 : 0)};
        }
        if (value instanceof double[]) {
            double[] arr = (double[]) value;
            byte[] bytes = new byte[arr.length * 8];
            for (int i = 0; i < arr.length; i++) {
                putLong(bytes, i * 8, Double.doubleToLongBits(arr[i]));
            }
            return bytes;
        }
        return value.toString().getBytes(StandardCharsets.UTF_8);
    }
 
    // ---- WPILOG binary format ------------------------------------------
 
    private void writeFileHeader() throws IOException {
        out.write("WPILOG".getBytes(StandardCharsets.US_ASCII));
        writeLE(0x0100, 2); // version 1.0, little-endian
        writeLE(0, 4); // extra header string length (none)
    }
 
    /** Control record (entry id 0): declares an entry's id, name and type. */
    private void writeStartRecord(int entryId, String name, String type, long timestampMicros)
            throws IOException {
        ByteArrayOutputStream payload = new ByteArrayOutputStream();
        payload.write(0); // control type: Start
        putU32(payload, entryId);
        putStr(payload, name);
        putStr(payload, type);
        putStr(payload, ""); // metadata (none)
        writeRecord(0, timestampMicros, payload.toByteArray());
    }
 
    /** Writes one record header + payload, using minimal-width length fields. */
    private void writeRecord(int entryId, long timestampMicros, byte[] payload) throws IOException {
        int idLen = byteCount(entryId & 0xFFFFFFFFL);
        int sizeLen = byteCount(payload.length);
        int tsLen = byteCount(timestampMicros);
 
        int bitfield = (idLen - 1) | ((sizeLen - 1) << 2) | ((tsLen - 1) << 4);
        out.write(bitfield);
        writeLE(entryId & 0xFFFFFFFFL, idLen);
        writeLE(payload.length, sizeLen);
        writeLE(timestampMicros, tsLen);
        out.write(payload);
    }
 
    private void writeLE(long value, int numBytes) throws IOException {
        for (int i = 0; i < numBytes; i++) {
            out.write((int) ((value >>> (8 * i)) & 0xFF));
        }
    }
 
    private static int byteCount(long value) {
        int n = 1;
        while ((value >>>= 8) != 0) {
            n++;
        }
        return n;
    }
 
    private static void putU32(ByteArrayOutputStream b, long value) {
        for (int i = 0; i < 4; i++) {
            b.write((int) ((value >>> (8 * i)) & 0xFF));
        }
    }
 
    private static void putStr(ByteArrayOutputStream b, String s) {
        byte[] bytes = s.getBytes(StandardCharsets.UTF_8);
        putU32(b, bytes.length);
        b.write(bytes, 0, bytes.length);
    }
 
    private static byte[] leDouble(double value) {
        byte[] bytes = new byte[8];
        putLong(bytes, 0, Double.doubleToLongBits(value));
        return bytes;
    }
 
    private static void putLong(byte[] dst, int offset, long bits) {
        for (int i = 0; i < 8; i++) {
            dst[offset + i] = (byte) ((bits >>> (8 * i)) & 0xFF);
        }
    }
}