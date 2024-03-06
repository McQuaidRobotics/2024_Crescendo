package monologue;

import edu.wpi.first.util.struct.StructSerializable;

public interface Logged {
  public default String getPath() {
    return "";
  }
  public default String getFullPath() {
    return Monologue.loggedRegistry.getOrDefault(this, "notfound");
  }

  public default void log(String key, boolean value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, boolean value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, int value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, int value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, long value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, long value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, float value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, float value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, double value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, double value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, String value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, String value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, byte[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, byte[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, boolean[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, boolean[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, int[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, int[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, long[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, long[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, float[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, float[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, double[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, double[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default void log(String key, String[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default void log(String key, String[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default <R extends StructSerializable> void log(String key, R value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default <R extends StructSerializable> void log(String key, R value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }

  public default <R extends StructSerializable> void log(String key, R[] value) {
    log(key, value, LogLevel.DEFAULT);
  }
  public default <R extends StructSerializable> void log(String key, R[] value, LogLevel level) {
    if (Monologue.isUnitTest()) return;
    Monologue.ntLogger.put(getFullPath() + "/" + key, value, level);
  }
}
