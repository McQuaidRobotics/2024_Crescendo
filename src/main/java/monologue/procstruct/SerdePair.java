package monologue.procstruct;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.struct.StructSerializable;

public class SerdePair<A, B> extends Pair<A, B> {
  @Retention(RetentionPolicy.RUNTIME)
  @Target({ ElementType.FIELD, ElementType.RECORD_COMPONENT })
  @Documented
  public @interface SerdePairHint {
    Class<?> a();
    Class<?> b();
  }

  private SerdePair(A a, B b) {
    super(a, b);
  }

  public static <A extends StructSerializable, B extends StructSerializable> SerdePair<A, B> of(A a, B b) {
    return new SerdePair<>(a, b);
  }

  public static <A extends Measure<?>, B extends Measure<?>> SerdePair<A, B> ofMeasure(A a, B b) {
    return new SerdePair<>(a, b);
  }
}
