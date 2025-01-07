package igknighters.subsystems.swerve.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import java.util.Arrays;
import monologue.ProceduralStructGenerator;
import monologue.ProceduralStructGenerator.FixedSizeArray;
import monologue.ProceduralStructGenerator.IgnoreStructField;

class SPGCalcs {
  private static final double kEpsilon = 1E-8;
  private static final int MAX_STEER_ITERATIONS = 8;
  private static final int MAX_DRIVE_ITERATIONS = 10;
  static final int NUM_MODULES = 4;

  static double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  @FunctionalInterface
  interface Function2d {
    double f(double x, double y);
  }

  /**
   * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
   * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
   * while being robust in ways that e.g. the Newton-Raphson method isn't.
   *
   * @param func The Function2d to take the root of.
   * @param x_0 x value of the lower bracket.
   * @param y_0 y value of the lower bracket.
   * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param x_1 x value of the upper bracket.
   * @param y_1 y value of the upper bracket.
   * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param iterations_left Number of iterations of root finding left.
   * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
   *     (approximate) root.
   */
  static double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));

    if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
      return s_guess;
    }

    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  static double findSteeringMaxS(
      double prevVx,
      double prevVy,
      double prevHeading,
      double desiredVx,
      double desiredVy,
      double desiredHeading,
      double maxDeviation) {
    desiredHeading = unwrapAngle(prevHeading, desiredHeading);
    double diff = desiredHeading - prevHeading;
    if (Math.abs(diff) <= maxDeviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = prevHeading + Math.signum(diff) * maxDeviation;
    Function2d func = (x, y) -> unwrapAngle(prevHeading, Math.atan2(y, x)) - offset;
    return findRoot(
        func,
        prevVx,
        prevVy,
        prevHeading - offset,
        desiredVx,
        desiredVy,
        desiredHeading - offset,
        MAX_STEER_ITERATIONS);
  }

  static double findDriveMaxS(
      double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_vel_step) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func = (x, y) -> Math.hypot(x, y) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, MAX_DRIVE_ITERATIONS);
  }

  static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
    return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
        && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
        && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  static boolean flipHeading(double prevToGoal) {
    return Math.abs(prevToGoal) > Math.PI / 2.0;
  }

  private static final Struct<LocalVectors> structVectors;
  private static final Struct<LocalVars> structVars;

  static {
    final var structVectorsProc =
        ProceduralStructGenerator.genObject(LocalVectors.class, LocalVectors::new);
    structVectors =
        new Struct<SPGCalcs.LocalVectors>() {
          @Override
          public String getSchema() {
            return "float64 vx;float64 vy;float64 cos;float64 sin;";
          }

          @Override
          public int getSize() {
            return 32;
          }

          @Override
          public Class<LocalVectors> getTypeClass() {
            return LocalVectors.class;
          }

          @Override
          public String getTypeName() {
            return "LocalVectors";
          }

          @Override
          public void pack(ByteBuffer bb, LocalVectors value) {
            bb.putDouble(value.vx);
            bb.putDouble(value.vy);
            bb.putDouble(value.cos);
            bb.putDouble(value.sin);
          }

          @Override
          public LocalVectors unpack(ByteBuffer bb) {
            return structVectorsProc.unpack(bb);
          }

          @Override
          public String toString() {
            return this.getTypeName() + "<" + this.getSize() + ">" + " {" + this.getSchema() + "}";
          }
        };
    structVars = ProceduralStructGenerator.genObject(LocalVars.class, LocalVars::new);
  }

  static final class LocalVectors implements StructSerializable {
    public double vx, vy, cos, sin;

    public LocalVectors() {}

    public void reset() {
      vx = vy = cos = sin = 0.0;
    }

    public void applyModuleState(SwerveModuleState state) {
      cos = state.angle.getCos();
      sin = state.angle.getSin();
      vx = cos * state.speedMetersPerSecond;
      vy = sin * state.speedMetersPerSecond;
      if (state.speedMetersPerSecond < 0.0) {
        applyRotation(Rotation2d.k180deg.getCos(), Rotation2d.k180deg.getSin());
      }
    }

    public LocalVectors applyRotation(double otherCos, double otherSin) {
      double newCos = cos * otherCos - sin * otherSin;
      double newSin = cos * otherSin + sin * otherCos;
      cos = newCos;
      sin = newSin;

      return this;
    }

    public double radians() {
      return Math.atan2(sin, cos);
    }

    public static final Struct<LocalVectors> struct = structVectors;
  }

  static final class LocalVars implements StructSerializable {
    @FixedSizeArray(size = NUM_MODULES)
    public LocalVectors[] prev;

    @FixedSizeArray(size = NUM_MODULES)
    public LocalVectors[] desired;

    public boolean needToSteer = true, allModulesShouldFlip = true;
    public double minS, dt;
    public double dx, dy, dtheta;
    public ChassisSpeeds prevSpeeds, desiredSpeeds;

    @FixedSizeArray(size = NUM_MODULES)
    public SwerveModuleState[] prevModuleStates;

    @FixedSizeArray(size = NUM_MODULES)
    public SwerveModuleState[] desiredModuleStates;

    @IgnoreStructField public Rotation2d[] steeringOverride;

    public LocalVars() {
      desiredSpeeds = prevSpeeds = new ChassisSpeeds();
      prev = new LocalVectors[NUM_MODULES];
      desired = new LocalVectors[NUM_MODULES];
      steeringOverride = new Rotation2d[NUM_MODULES];
      for (int i = 0; i < NUM_MODULES; i++) {
        prev[i] = new LocalVectors();
        desired[i] = new LocalVectors();
      }
    }

    public LocalVars reset() {
      needToSteer = allModulesShouldFlip = true;
      Arrays.fill(steeringOverride, null);
      for (int i = 0; i < NUM_MODULES; i++) {
        prev[i].reset();
        desired[i].reset();
      }

      return this;
    }

    public static final Struct<LocalVars> struct = structVars;
  }
}
