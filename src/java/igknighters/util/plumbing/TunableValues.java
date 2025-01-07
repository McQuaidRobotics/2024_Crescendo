package igknighters.util.plumbing;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import java.util.HashMap;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

/**
 * An api for creating NT tunable values. Bools, Ints and Doubles are supported.
 *
 * @see TunableValues#getDouble(String, double) TunableValues.getDouble()
 * @see TunableValues#getInteger(String, int) TunableValues.getInteger()
 * @see TunableValues#getBoolean(String, boolean) TunableValues.getBoolean()
 */
public class TunableValues {
  private static final String pathPrefix = "/Tunables/";

  private static final HashMap<String, TunableDouble> doubleTunables = new HashMap<>();
  private static final HashMap<String, TunableInteger> intTunables = new HashMap<>();
  private static final HashMap<String, TunableBoolean> boolTunables = new HashMap<>();

  /**
   * An object representing a tunable double value.
   *
   * @see TunableValues#getDouble(String, double) Creating a tunable double
   */
  public static class TunableDouble implements Cloneable {
    private final DoubleSubscriber subscriber;
    private final double defaultValue;

    private TunableDouble(String path, double defaultValue) {
      DoubleEntry entry =
          NetworkTableInstance.getDefault()
              .getDoubleTopic(pathPrefix + path)
              .getEntry(defaultValue, PubSubOption.pollStorage(1), PubSubOption.excludeSelf(true));
      entry.setDefault(defaultValue);
      this.subscriber = entry;
      this.defaultValue = defaultValue;
    }

    private TunableDouble(DoubleSubscriber subscriber, double defaultValue) {
      this.subscriber = subscriber;
      this.defaultValue = defaultValue;
    }

    /**
     * Get the value of the tunable, returns the default value a value is not present.
     *
     * @return The value of the tunable
     */
    public double value() {
      return subscriber.get(defaultValue);
    }

    /**
     * Gets the new value if the tunable has been changed.
     *
     * @return The new value if available
     */
    public OptionalDouble newValue() {
      var vals = subscriber.readQueueValues();
      if (vals.length > 0) {
        return OptionalDouble.of(vals[0]);
      }
      return OptionalDouble.empty();
    }

    @Override
    public TunableDouble clone() {
      return new TunableDouble(subscriber, defaultValue);
    }
  }

  /**
   * Get a tunable double value.
   *
   * <p>Tunable values are cached, so calling this method with the same path will return the same
   * object and not create a new one (throwing out the new default value if it was different).
   *
   * @param path The path to the tunable value (it will be prefixed with "/Tunables/")
   * @param defaultValue The default value of the tunable
   * @return The tunable value object
   * @see TunableDouble#value() How to get a value from the object
   */
  public static TunableDouble getDouble(String path, double defaultValue) {
    if (doubleTunables.containsKey(path)) {
      return doubleTunables.get(path);
    } else {
      TunableDouble newTunable = new TunableDouble(path, defaultValue);
      doubleTunables.put(path, newTunable);
      return newTunable;
    }
  }

  /**
   * An object representing a tunable integer value.
   *
   * @see TunableValues#getInteger(String, int) Creating a tunable integer
   */
  public static class TunableInteger implements Cloneable {
    private final IntegerEntry entry;
    private final int defaultValue;

    private TunableInteger(String path, int defaultValue) {
      this.entry =
          NetworkTableInstance.getDefault()
              .getIntegerTopic(pathPrefix + path)
              .getEntry(defaultValue, PubSubOption.pollStorage(1), PubSubOption.excludeSelf(true));
      entry.setDefault(defaultValue);
      this.defaultValue = defaultValue;
    }

    private TunableInteger(IntegerEntry entry, int defaultValue) {
      this.entry = entry;
      this.defaultValue = defaultValue;
    }

    /**
     * Get the value of the tunable, returns the default value a value is not present.
     *
     * @return The value of the tunable
     */
    public int value() {
      return (int)
          Math.min(Math.max(entry.get(defaultValue), Integer.MIN_VALUE), Integer.MAX_VALUE);
    }

    /**
     * Gets the new value if the tunable has been changed.
     *
     * @return The new value if available
     */
    public OptionalInt newValue() {
      var vals = entry.readQueueValues();
      if (vals.length > 0) {
        return OptionalInt.of(
            (int) Math.min(Math.max(vals[0], Integer.MIN_VALUE), Integer.MAX_VALUE));
      }
      return OptionalInt.empty();
    }

    @Override
    public TunableInteger clone() {
      return new TunableInteger(entry, defaultValue);
    }
  }

  /**
   * Get a tunable integer value.
   *
   * <p>Tunable values are cached, so calling this method with the same path will return the same
   * object and not create a new one (throwing out the new default value if it was different).
   *
   * @param path The path to the tunable value (it will be prefixed with "/Tunables/")
   * @param defaultValue The default value of the tunable
   * @return The tunable value object
   * @see TunableInteger#value() How to get a value from the object
   */
  public static TunableInteger getInteger(String path, int defaultValue) {
    if (intTunables.containsKey(path)) {
      return intTunables.get(path);
    } else {
      TunableInteger newTunable = new TunableInteger(path, defaultValue);
      intTunables.put(path, newTunable);
      return newTunable;
    }
  }

  /**
   * An object representing a tunable boolean value.
   *
   * @see TunableValues#getBoolean(String, boolean) Creating a tunable boolean
   */
  public static class TunableBoolean implements Cloneable {
    private final BooleanEntry entry;
    private final boolean defaultValue;

    private TunableBoolean(String path, boolean defaultValue) {
      this.entry =
          NetworkTableInstance.getDefault()
              .getBooleanTopic(pathPrefix + path)
              .getEntry(defaultValue);
      entry.setDefault(defaultValue);
      this.defaultValue = defaultValue;
    }

    private TunableBoolean(BooleanEntry entry, boolean defaultValue) {
      this.entry = entry;
      this.defaultValue = defaultValue;
    }

    /**
     * Get the value of the tunable, returns the default value if a value is not present.
     *
     * @return The value of the tunable
     */
    public boolean value() {
      return entry.get(defaultValue);
    }

    /**
     * Gets the new value if the tunable has been changed.
     *
     * @return The new value if available
     */
    public Optional<Boolean> newValue() {
      var vals = entry.readQueueValues();
      if (vals.length > 0) {
        return Optional.of(vals[0]);
      }
      return Optional.empty();
    }

    @Override
    public TunableBoolean clone() {
      return new TunableBoolean(entry, defaultValue);
    }
  }

  /**
   * Get a tunable boolean value.
   *
   * <p>Tunable values are cached, so calling this method with the same path will return the same
   * object and not create a new one (throwing out the new default value if it was different).
   *
   * @param path The path to the tunable value (it will be prefixed with "/Tunables/")
   * @param defaultValue The default value of the tunable
   * @return The tunable value object
   * @see TunableBoolean#value() How to get a value from the object
   */
  public static TunableBoolean getBoolean(String path, boolean defaultValue) {
    if (boolTunables.containsKey(path)) {
      return boolTunables.get(path);
    } else {
      TunableBoolean newTunable = new TunableBoolean(path, defaultValue);
      boolTunables.put(path, newTunable);
      return newTunable;
    }
  }
}
