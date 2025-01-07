package monologue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import monologue.LoggingTree.StaticObjectNode;

/**
 * The Monologue class is the main entry point for the Monologue library. It is responsible for
 * setting up the Monologue library, updating the loggers, and logging objects.
 *
 * <p>Monologue is a library that allows for easy logging of objects to NetworkTables and Datalog.
 * It has {@link Annotations} that allow implicit logging of fields and methods on objects that
 * implement the {@link Logged} interface.
 *
 * <p>Monologue works by creating a tree of objects that implement the {@link Logged} interface and
 * then logging the fields and methods of those objects to NetworkTables and Datalog based on their
 * annotations. For example let's say the root object is {@code Robot.java}, you would implemenet
 * {@link Logged} on the root object and then call {@link #setupMonologue(Logged, String,
 * MonologueConfig)} with the root object and a root path (typically "/Robot"). This will recurse
 * through all the fields in {@code RobotContainer.java} and search for more objects that implement
 * {@link Logged} and repeat the process until all fields and methods have been logged.
 *
 * <p>Monologue has a rich error handling system that will tell you what you did wrong and where you
 * did it wrong. If you would like to run Monologue in whole robot Unit Tests you can use {@link
 * #setupMonologueDisabled(Logged, String, boolean)} to disable logging and only run the error
 * checking.
 *
 * <p><b>WARNING:</b> Any use of `DatalogManager` before Monologue.setupMonologue() is undefined
 * behavior and can result in a crash.
 */
public class Monologue extends GlobalLogged {
  /** The Monologue library wide OPTIMIZE_BANDWIDTH flag, is used to divert logging */
  private static boolean OPTIMIZE_BANDWIDTH = true;

  private static MonologueConfig config = new MonologueConfig();

  private static boolean HAS_SETUP_BEEN_CALLED = false;
  private static boolean IS_DISABLED = false;
  private static boolean THROW_ON_WARN = false;

  private static final ArrayList<Runnable> prematureCalls = new ArrayList<Runnable>();
  private static final ArrayList<StaticObjectNode> trees = new ArrayList<StaticObjectNode>();

  /**
   * An object to hold the configuration for the Monologue library. This allows for easier default
   * values, more readable code, and ability to add more configuration later without breaking
   * existing code.
   */
  public static record MonologueConfig(
      BooleanSupplier optimizeBandwidthSupplier,
      String datalogPrefix,
      boolean throwOnWarn,
      boolean allowNonFinalLoggedFields) {
    public MonologueConfig {
      if (optimizeBandwidthSupplier == null) {
        RuntimeLog.warn(
            "shouldOptimizeBandwidthSupplier cannot be null in MonologueConfig, falling back to false (always log NT)");

        optimizeBandwidthSupplier = () -> false;
      }
      if (datalogPrefix == null) {
        RuntimeLog.warn("datalogPrefix cannot be null in MonologueConfig, falling back to \"NT:\"");
        datalogPrefix = "NT:";
      }
    }

    public MonologueConfig() {
      this(() -> false, "NT:", false, false);
    }

    /**
     * Updates the OptimizeBandwidth flag supplier.
     *
     * @param optimizeBandwidth The new OptimizeBandwidth flag supplier
     * @return A new MonologueConfig object with the updated OptimizeBandwidth flag supplier
     */
    public MonologueConfig withOptimizeBandwidth(BooleanSupplier optimizeBandwidth) {
      return new MonologueConfig(
          optimizeBandwidth, datalogPrefix, throwOnWarn, allowNonFinalLoggedFields);
    }

    /**
     * Updates the OptimizeBandwidth static flag.
     *
     * @param optimizeBandwidth The new OptimizeBandwidth flag
     * @return A new MonologueConfig object with the updated OptimizeBandwidth flag
     */
    public MonologueConfig withOptimizeBandwidth(boolean optimizeBandwidth) {
      return new MonologueConfig(
          () -> optimizeBandwidth, datalogPrefix, throwOnWarn, allowNonFinalLoggedFields);
    }

    /**
     * Updates the lazyLogging flag.
     *
     * @param lazyLogging The new lazyLogging flag
     * @return A new MonologueConfig object with the updated lazyLogging flag
     */
    public MonologueConfig withLazyLogging(boolean lazyLogging) {
      return new MonologueConfig(
          optimizeBandwidthSupplier, datalogPrefix, throwOnWarn, allowNonFinalLoggedFields);
    }

    /**
     * Updates the datalogPrefix.
     *
     * @param datalogPrefix The new datalogPrefix
     * @return A new MonologueConfig object with the updated datalogPrefix
     */
    public MonologueConfig withDatalogPrefix(String datalogPrefix) {
      return new MonologueConfig(
          optimizeBandwidthSupplier, datalogPrefix, throwOnWarn, allowNonFinalLoggedFields);
    }

    /**
     * Updates the throwOnWarn flag. If true, Monologue will throw an exception when a Monologue
     * internal warning is emitted. This is useful for catching issues in CI / Unit Tests.
     *
     * @param throwOnWarn The new throwOnWarn flag
     * @return A new MonologueConfig object with the updated throwOnWarn flag
     */
    public MonologueConfig withThrowOnWarning(boolean throwOnWarn) {
      return new MonologueConfig(
          optimizeBandwidthSupplier, datalogPrefix, throwOnWarn, allowNonFinalLoggedFields);
    }

    /**
     * Updates the allowNonFinalLoggedFields flag. If true, Monologue will allow non-final fields
     * containing {@link Logged} objects to be logged. This is not reccomended as it can lead to
     * unexpected behavior.
     *
     * @param allowNonFinalLoggedFields The new allowNonFinalLoggedFields flag
     * @return A new MonologueConfig object with the updated allowNonFinalLoggedFields flag
     */
    public MonologueConfig withAllowNonFinalLoggedFields(boolean allowNonFinalLoggedFields) {
      return new MonologueConfig(
          optimizeBandwidthSupplier, datalogPrefix, throwOnWarn, allowNonFinalLoggedFields);
    }
  }

  /**
   * Is the main entry point for the monologue library. It will interate over every member of the
   * provided Logged object and evaluated if it should be logged to the network tables or to a file.
   *
   * <p>Will also recursively check field values for classes that implement Logged and log those as
   * well.
   *
   * @param loggable the root Logged object to log
   * @param rootpath the root path to log to\
   * @param config the configuration for the Monologue library
   * @apiNote Should only be called once, if another {@link Logged} tree needs to be created use
   *     {@link #logTree(Logged, String)} for additional trees
   */
  public static void setupMonologue(Logged loggable, String rootpath, MonologueConfig config) {
    if (HAS_SETUP_BEEN_CALLED) {
      RuntimeLog.warn(
          "Monologue.setupMonologue() has already been called, further calls will do nothing");
      return;
    }

    NetworkTableInstance.getDefault()
        .startEntryDataLog(DataLogManager.getLog(), "", config.datalogPrefix);

    // create and start a timer to time the setup process
    Timer timer = new Timer();
    timer.start();

    Monologue.config = config;
    HAS_SETUP_BEEN_CALLED = true;
    rootpath = NetworkTable.normalizeKey(rootpath, true);
    Monologue.setRootPath(rootpath);
    RuntimeLog.info(
        "Monologue.setupMonologue() called on "
            + loggable.getClass().getName()
            + " with rootpath "
            + rootpath
            + " and config"
            + config);

    THROW_ON_WARN = config.throwOnWarn;

    OPTIMIZE_BANDWIDTH = config.optimizeBandwidthSupplier.getAsBoolean();

    logTree(loggable, rootpath);

    prematureCalls.forEach(Runnable::run);
    prematureCalls.clear();

    System.gc();

    RuntimeLog.info("Monologue.setupMonologue() finished in " + timer.get() + " seconds");
  }

  /**
   * Sets up Monologue in a disabled state, will not log anything.
   *
   * <p>This can be helpful for applications like unit tests where you want to validate Monoluge
   * logic and logging types without actually logging anything.
   *
   * <p>This method can also be called multiple times, this allows this to be called multiple times
   * in one unit test session without throwing an exception.
   *
   * @param loggable the root Logged object to log
   * @param rootpath the root path to log to
   * @param throwOnWarn if true, will throw an exception when a Monologue internal warning is
   *     emitted
   */
  public static void setupMonologueDisabled(Logged loggable, String rootpath, boolean throwOnWarn) {
    if (HAS_SETUP_BEEN_CALLED && !IS_DISABLED) {
      RuntimeLog.warn(
          "Monologue.setupMonologue() has already been called, disabling after setup will do nothing");
      return;
    }

    HAS_SETUP_BEEN_CALLED = true;
    IS_DISABLED = true;
    THROW_ON_WARN = throwOnWarn;

    RuntimeLog.info(
        "Monologue.setupMonologueDisabled() called on "
            + loggable.getClass().getName()
            + " with rootpath "
            + rootpath);

    // wont actually log anything, will just do state and type validation to provide use in CI/unit
    // tests
    logTree(loggable, rootpath);

    Logged.registry.clear();

    RuntimeLog.info("Monologue.setupMonologueDisabled() finished");
  }

  /**
   * Creates a logging tree for the provided {@link Logged} object. Will also recursively check
   * field values for classes that implement {@link Logged} and log those as well.
   *
   * @param loggable the obj to scrape
   * @param path the path to log to
   * @throws IllegalStateException If {@link #setupMonologue()} or {@link #setupMonologueDisabled()}
   *     is not called first
   */
  public static void logTree(Logged loggable, String path) {
    if (!hasBeenSetup())
      throw new IllegalStateException(
          "Tried to use Monologue.logTree() before using a Monologue setup method");

    if (path == null || path.isEmpty()) {
      RuntimeLog.warn("Invalid path for Monologue.logTree(): " + path);
      return;
    } else if (path == "/") {
      RuntimeLog.warn("Root path of / is not allowed for Monologue.logTree()");
      return;
    }
    RuntimeLog.info(
        "Monologue.logTree() called on " + loggable.getClass().getName() + " with path " + path);

    StaticObjectNode node = new LoggingTree.StaticObjectNode(path, loggable);
    Eval.exploreNodes(Eval.getLoggedClasses(loggable.getClass()), node);
    Logged.addNode(loggable, node);

    trees.add(node);

    updateAll();
  }

  /**
   * Updates all the loggers, ideally called every cycle.
   *
   * @apiNote Should only be called on the same thread monologue was setup on
   */
  public static void updateAll() {
    if (isMonologueDisabled()) return;
    if (!hasBeenSetup()) RuntimeLog.warn("Called Monologue.updateAll before Monologue was setup");
    boolean newOptimizeBandwidth = config.optimizeBandwidthSupplier.getAsBoolean();
    if (newOptimizeBandwidth != OPTIMIZE_BANDWIDTH) {
      RuntimeLog.info("Monologue.updateAll() updated FILE_ONLY flag to " + newOptimizeBandwidth);
      log("MonologueOptimizeBandwidth", newOptimizeBandwidth);
    }
    OPTIMIZE_BANDWIDTH = newOptimizeBandwidth;
    MonoSendableLayer.updateAll();
    for (StaticObjectNode tree : trees) {
      tree.log(null);
    }
  }

  static void prematureLog(Runnable runnable) {
    prematureCalls.add(runnable);
  }

  /**
   * Checks if the Monologue library is in file only mode.
   *
   * @return true if Monologue is in file only mode, false otherwise
   */
  static boolean isBandwidthOptimizationEnabled() {
    return OPTIMIZE_BANDWIDTH;
  }

  /**
   * Checks if the Monologue library is disabled.
   *
   * @return true if Monologue is disabled, false otherwise
   * @apiNote This is useful for unit tests where you want to validate Monologue logic and logging
   */
  static boolean isMonologueDisabled() {
    return IS_DISABLED;
  }

  /**
   * Checks if the Monologue library has been setup.
   *
   * @return true if Monologue has been setup, false otherwise
   */
  static boolean hasBeenSetup() {
    return HAS_SETUP_BEEN_CALLED;
  }

  /**
   * Checks if the Monologue library should throw an exception when a Monologue internal warning is
   * emitted.
   *
   * @return true if Monologue should throw an exception on warning, false otherwise
   */
  static boolean shouldThrowOnWarn() {
    return THROW_ON_WARN;
  }

  /**
   * Checks if the Monologue library is ready to log. If it is not ready, it will log a warning
   * using the key provided.
   *
   * @param key The key to log if Monologue is not ready
   * @return true if Monologue is ready, false otherwise
   */
  static boolean isMonologueReady(String key) {
    if (!hasBeenSetup()) {
      RuntimeLog.warn("Tried to log \"" + key + "\" before Monologue was setup");
      return false;
    }
    return true;
  }
}
