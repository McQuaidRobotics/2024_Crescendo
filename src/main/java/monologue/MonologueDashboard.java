package monologue;

import java.util.Optional;

import edu.wpi.first.util.struct.StructSerializable;

public class MonologueDashboard {

    private static Optional<GenericLogger> getLogger(LogLevel level) {
        boolean fileOnly = Monologue.isFileOnly();
        if (level.equals(LogLevel.DEFAULT)) {
            return fileOnly ? Optional.of(Monologue.dataLogger) : Optional.of(Monologue.ntLogger);
        } else if (level.equals(LogLevel.NOT_FILE_ONLY)) {
            return fileOnly ? Optional.empty() : Optional.of(Monologue.ntLogger);
        } else {
            return Optional.of(Monologue.ntLogger);
        }
    }

    public static void put(String entryName, boolean value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, boolean value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, int value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, int value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, long value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, long value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, float value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, float value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, double value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, double value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, String value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, String value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, byte[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, byte[] value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, boolean[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, boolean[] value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, int[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, int[] value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, float[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, float[] value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, double[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, double[] value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static void put(String entryName, String[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static void put(String entryName, String[] value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static <R extends StructSerializable> void put(String entryName, R value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static <R extends StructSerializable> void put(String entryName, R value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }

    public static <R extends StructSerializable> void put(String entryName, R[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }

    public static <R extends StructSerializable> void put(String entryName, R[] value, LogLevel level) {
        getLogger(level).ifPresent(logger -> logger.put(entryName, value));
    }
}
