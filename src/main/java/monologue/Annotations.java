package monologue;

import java.lang.annotation.Annotation;
import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@SuppressWarnings("unchecked")
public class Annotations {
  static final Class<? extends Annotation>[] ALL_ANNOTATIONS =
      new Class[] {
        Log.class,
        Log.Once.class
      };

  /**
   * Logs the annotated field/method to NetworkTables if inside a {@link Logged} class.
   *
   * <p>Static fields and methods will emit a warning and not be logged.
   *
   * @param key [optional] the key to log the variable as. If empty, the key will be the name of the
   *     field/method
   * @param sink [optional] the log sink to use
   */
  @Documented
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.FIELD, ElementType.METHOD})
  public @interface Log {

    /** The relative path to log to. If empty, the path will be the name of the field/method. */
    public String key() default "";

    /**
     * The {@link LogSink} to use.
     */
    public LogSink sink() default LogSink.NT;

    /**
     * Logs the annotated field/method to NetworkTables if inside a {@link Logged} class.
     *
     * @param key [optional] the key to log the variable as. If empty, the key will be the name of
     *     the field/method
     */
    @Documented
    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    public @interface Once {
      /** The relative path to log to. If empty, the path will be the name of the field/method. */
      public String key() default "";

      /**
       * The {@link LogSink} to use.
       */
      public LogSink sink() default LogSink.NT;
    }
  }

  /**
   * Makes the annotated field containing a {@link Logged} class not be recursed into.
   *
   * @apiNote this will also make fields inside the object in the field not be logged
   */
  @Documented
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.FIELD})
  public @interface IgnoreLogged {}

  /**
   * Allows singletons to be logged only once with a predefined key.
   * 
   * <p>This also allows static variables to be logged under the singleton's key.
   * 
   * @param key the key to log at, still appends the class name
   */
  @Documented
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.TYPE})
  public @interface SingletonLogged {
    public String key();
  }

  /**
   * Will cause the internal fields of the annotated field to be logged as if they were fields of
   * the object this field is in. This is useful for flattening complex objects into a single path.
   */
  @Documented
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.FIELD})
  public @interface FlattenedLogged {}

  /**
   * Will make Monologue aware that this field could contain an
   * object that implements {@link Logged} but the type of the field itself
   * does not implement {@link Logged}.
   */
  @Documented
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ElementType.FIELD})
  public @interface MaybeLoggedType {}
}
