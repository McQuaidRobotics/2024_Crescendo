package monologue;

import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import monologue.LoggingTree.LoggingNode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.WeakHashMap;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.struct.Struct;

/**
 * Interface for classes that can hold {@link Monologue} annotated fields for
 * {@link Monologue#setupMonologue} and {@link Monologue#logObj} to log.
 * 
 * This class also allows for an imperative way to log values with the {@link #log} methods.
 * 
 * Class fields that hold {@code Logged} objects should be final.
 * 
 * @see Monologue
 * @see Annotations.Log
 */
public interface Logged {

  static final WeakHashMap<Logged, ArrayList<LoggingNode>> registry = new WeakHashMap<>();
  static final HashMap<Class<?>, LoggingNode> singletons = new HashMap<>();

  static void addNode(Logged logged, LoggingNode node) {
    var lst = getNodes(logged);
    if (!lst.contains(node)) {
      lst.add(node);
    }
  }

  static void addSingleton(Class<?> logged, LoggingNode node) {
    singletons.put(logged, node);
  }

  static boolean singletonAlreadyAdded(Class<?> logged) {
    return singletons.containsKey(logged);
  }

  static List<LoggingNode> getNodes(Logged logged) {
    registry.putIfAbsent(logged, new ArrayList<>());
    return registry.get(logged);
  }

  /**
   * Normally the name of {@code this} in the object tree is based off the field in the object the reference is stored in.
   * Overriding this method allows you to specify a different name for the object in the object tree.
   * 
   * If the returnd string has a '/' in it, the object will be placed in a subtable.
   *
   * @return The name of the object in the object tree.
   */
  public default String getOverrideName() {
    return "";
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default boolean  log(String key, boolean value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default boolean log(String key, boolean value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default int  log(String key, int value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default int log(String key, int value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default long  log(String key, long value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default long log(String key, long value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default float  log(String key, float value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default float log(String key, float value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default double  log(String key, double value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default double log(String key, double value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default String  log(String key, String value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default String log(String key, String value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default byte[]  log(String key, byte[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default byte[] log(String key, byte[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default boolean[]  log(String key, boolean[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default boolean[] log(String key, boolean[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default int[]  log(String key, int[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default int[] log(String key, int[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default long[]  log(String key, long[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default long[] log(String key, long[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default float[]  log(String key, float[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default float[] log(String key, float[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default double[]  log(String key, double[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default double[] log(String key, double[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default String[]  log(String key, String[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default String[] log(String key, String[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R extends StructSerializable> R log(String key, R value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default <R extends StructSerializable> R log(String key, R value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R extends StructSerializable> R[] log(String key, R[] value) {
    return log(key, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default <R extends StructSerializable> R[] log(String key, R[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R> R log(String key, Struct<R> struct, R value) {
    return log(key, struct, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default <R> R log(String key, Struct<R> struct, R value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, struct, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, struct, value, sink);
    }
    return value;
  }

  /**
   * Logs a value with the default log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   */
  public default <R> R[] log(String key, Struct<R> struct, R[] value) {
    return log(key, struct, value, LogSink.NT);
  }

  /**
   * Logs a value with the specified log sink.
   * The key is relative to the objects path this is being called in.
   * 
   * @param key The key to log the value under relative to the objects path.
   * @param value The value to log.
   * @param sink The log sink to log the value under.
   */
  public default <R> R[] log(String key, Struct<R> struct, R[] value, LogSink sink) {
    if (!Monologue.hasBeenSetup()) {
      Monologue.prematureLog(() -> log(key, struct, value, sink));
      return value;
    }
    String slashkey = "/" + key;
    for (LoggingNode node : getNodes(this)) {
      Monologue.log(node.getPath() + slashkey, struct, value, sink);
    }
    return value;
  }

  /**
    * Logs a Sendable using the Monologue machinery.
    * 
    * @param entryName The name of the entry to log, this is an absolute path.
    * @param value The value to log.
    */
    public static void publishSendable(String entryName, Sendable value, LogSink sink) {
      if (!Monologue.hasBeenSetup()) {
        Monologue.prematureLog(() -> publishSendable(entryName, value, sink));
        return;
      }
      Monologue.publishSendable(entryName, value, sink);
    }

      /**
        * Logs a Sendable using the Monologue machinery.
        * 
        * @param entryName The name of the entry to log, this is an absolute path.
        * @param value The value to log.
        */
      public static void publishSendable(String entryName, Field2d value, LogSink sink) {
        if (!Monologue.hasBeenSetup()) {
          Monologue.prematureLog(() -> publishSendable(entryName, value, sink));
          return;
        }
        Monologue.publishSendable(entryName, value, sink);
      }

      /**
        * Logs a Sendable using the Monologue machinery.
        * 
        * @param entryName The name of the entry to log, this is an absolute path.
        * @param value The value to log.
        */
      public static void publishSendable(String entryName, Mechanism2d value, LogSink sink) {
        if (!Monologue.hasBeenSetup()) {
          Monologue.prematureLog(() -> publishSendable(entryName, value, sink));
          return;
        }
        Monologue.publishSendable(entryName, value, sink);
      }
}
