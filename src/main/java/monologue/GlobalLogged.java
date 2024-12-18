package monologue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import monologue.MonologueSendableLayer.NtSendableCompat;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * The GlobalLogged class is a utility class that provides a simple way to use Monologue's
 * logging tooling from any part of your robot code. It provides a set of log methods that
 * allow you to log data to the NetworkTables and DataLog.
 * 
 * @see Monologue
 * @see LogSink
 */
class GlobalLogged {
  static String ROOT_PATH = "";

  static void setRootPath(String rootPath) {
    ROOT_PATH = NetworkTable.normalizeKey(rootPath, true);
  }

  /**
    * Logs a boolean using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static boolean log(String entryName, boolean value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a boolean using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static boolean log(String entryName, boolean value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, boolean.class, sink).log(value);

    return value;
  }

  /**
    * Logs a int using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static int log(String entryName, int value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a int using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static int log(String entryName, int value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, int.class, sink).log(value);

    return value;
  }

  /**
    * Logs a long using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static long log(String entryName, long value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a long using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static long log(String entryName, long value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, long.class, sink).log(value);

    return value;
  }

  /**
    * Logs a float using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static float log(String entryName, float value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a float using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static float log(String entryName, float value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, float.class, sink).log(value);

    return value;
  }

  /**
    * Logs a double using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static double log(String entryName, double value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a double using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static double log(String entryName, double value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, double.class, sink).log(value);

    return value;
  }

  /**
    * Logs a String using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static String log(String entryName, String value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a String using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static String log(String entryName, String value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, String.class, sink).log(value);

    return value;
  }

  /**
    * Logs a byte[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static byte[] log(String entryName, byte[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a byte[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static byte[] log(String entryName, byte[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, byte[].class, sink).log(value);

    return value;
  }

  /**
    * Logs a boolean[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static boolean[] log(String entryName, boolean[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a boolean[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static boolean[] log(String entryName, boolean[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, boolean[].class, sink).log(value);

    return value;
  }

  /**
    * Logs a int[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static int[] log(String entryName, int[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a int[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static int[] log(String entryName, int[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, int[].class, sink).log(value);

    return value;
  }

  /**
    * Logs a long[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static long[] log(String entryName, long[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a long[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static long[] log(String entryName, long[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, long[].class, sink).log(value);

    return value;
  }

  /**
    * Logs a float[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static float[] log(String entryName, float[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a float[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static float[] log(String entryName, float[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, float[].class, sink).log(value);

    return value;
  }

  /**
    * Logs a double[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static double[] log(String entryName, double[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a double[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static double[] log(String entryName, double[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, double[].class, sink).log(value);

    return value;
  }

  /**
    * Logs a String[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static String[] log(String entryName, String[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a String[] using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  public static String[] log(String entryName, String[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    MonologueEntryLayer.MonologueEntry.create(entryName, String[].class, sink).log(value);

    return value;
  }


  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static <R extends StructSerializable> R log(String entryName, R value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  @SuppressWarnings("unchecked")
  public static <R extends StructSerializable> R log(String entryName, R value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    Class<R> clazz = (Class<R>) value.getClass();
    Struct<R> struct = ProceduralStructGenerator.extractClassStruct(clazz).get();
    MonologueEntryLayer.MonologueEntry.create(entryName, struct, clazz, sink).log(value);

    return value;
  }

  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static <R extends StructSerializable> R[] log(String entryName, R[] value) {
    return log(entryName, value, LogSink.NT);
  }
  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  @SuppressWarnings("unchecked")
  public static <R extends StructSerializable> R[] log(String entryName, R[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, value, sink));
      return value;
    }
    Class<R[]> clazz = (Class<R[]>) value.getClass();
    Struct<R> struct = (Struct<R>) ProceduralStructGenerator.extractClassStructDynamic(clazz.getComponentType()).get();
    MonologueEntryLayer.MonologueEntry.createStructArray(entryName, struct, clazz, sink).log(value);

    return value;
  }

  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    */
  public static <R> R log(String entryName, Struct<R> struct, R value) {
    return log(entryName, struct, value, LogSink.NT);
  }
  /**
    * Logs a Serializable Struct using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  @SuppressWarnings("unchecked")
  public static <R> R log(String entryName, Struct<R> struct, R value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, struct, value, sink));
      return value;
    }
    Class<R> clazz = (Class<R>) value.getClass();
    MonologueEntryLayer.MonologueEntry.create(entryName, struct, clazz, sink).log(value);

    return value;
  }

  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    */
  public static <R> R[] log(String entryName, Struct<R> struct, R[] value) {
    return log(entryName, struct, value, LogSink.NT);
  }
  /**
    * Logs an array of Serializable Structs using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param struct The struct type to log.
    * @param value The value to log.
    * @param sink The log sink to use.
    */
  @SuppressWarnings("unchecked")
  public static <R> R[] log(String entryName, Struct<R> struct, R[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) {
      String entryNameFinal = entryName;
      Monologue.prematureLog(() -> GlobalLogged.log(entryNameFinal, struct, value, sink));
      return value;
    }
    Class<R[]> clazz = (Class<R[]>) value.getClass();
    MonologueEntryLayer.MonologueEntry.createStructArray(entryName, struct, clazz, sink).log(value);

    return value;
  }

  static <R> R[] logStructArray(String entryName, Struct<R> struct, R[] value, LogSink sink) {
    return log(entryName, struct, value, sink);
  }

  /**
    * Logs a Sendable using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
  public static void publishSendable(String entryName, Sendable value, LogSink sink) {
    if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) return;
    entryName = NetworkTable.normalizeKey(entryName, true);
    var builder = new MonologueSendableLayer.Builder(entryName, sink);
    value.initSendable(builder);
  }

    /**
      * Logs a Sendable using the Monologue machinery.
      * 
      * @param entryName The name of the entry to log, this is an absolute path.
      * @param value The value to log.
      */
    public static void publishSendable(String entryName, Field2d value, LogSink sink) {
      if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) return;
      entryName = NetworkTable.normalizeKey(entryName, true);
      NtSendableCompat.addField2d(entryName, value, sink);
    }

    /**
      * Logs a Sendable using the Monologue machinery.
      * 
      * @param entryName The name of the entry to log, this is an absolute path.
      * @param value The value to log.
      */
    public static void publishSendable(String entryName, Mechanism2d value, LogSink sink) {
      if (!Monologue.hasBeenSetup() || Monologue.isMonologueDisabled()) return;
      entryName = NetworkTable.normalizeKey(entryName, true);
      NtSendableCompat.addMechanism2d(entryName, value, sink);
    }
}
