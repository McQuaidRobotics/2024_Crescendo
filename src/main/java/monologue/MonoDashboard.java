package monologue;

import edu.wpi.first.util.struct.StructSerializable;

public class MonoDashboard {


    public static void put(String entryName, boolean value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, boolean value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, int value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, int value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, long value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, long value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, float value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, float value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, double value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, double value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, String value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, String value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, byte[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, byte[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, boolean[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, boolean[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, int[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, int[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, long[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, long[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, float[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, float[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, double[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, double[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static void put(String entryName, String[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public static void put(String entryName, String[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }


    public static <R extends StructSerializable> void put(String entryName, R value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static <R extends StructSerializable> void put(String entryName, R value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }

    public static <R extends StructSerializable> void put(String entryName, R[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static <R extends StructSerializable> void put(String entryName, R[] value, LogLevel level) {
        if (Monologue.isUnitTest()) return;
        Monologue.ntLogger.put(entryName, value, level);
    }
}
