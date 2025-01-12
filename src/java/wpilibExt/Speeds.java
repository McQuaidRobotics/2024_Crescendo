package wpilibExt;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import monologue.ProceduralStructGenerator;

/**
 * An interface to add type safety to the frame of reference of speeds.
 *
 * <p>Speeds can come in two variants for different <a
 * href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">frame
 * of references</a>:
 *
 * <ul>
 *   <li>Field relative: The speeds are relative to the field coordinate system.
 *   <li>Robot relative: The speeds are relative to the robot coordinate system.
 * </ul>
 */
public sealed interface Speeds extends StructSerializable {
  /**
   * Speeds in the field coordinate system.
   *
   * <ul>
   *   <li>Positive x is towards the away from the blue alliance wall.
   *   <li>Positive y is left from the perspective of a blue alliance driver.
   *   <li>Positive omega is counter-clockwise.
   *   <li>(0, 0) is the corner between the blue alliance wall and the wall adjacent in the -y
   *       direction.
   * </ul>
   */
  public record FieldSpeeds(double vx, double vy, double omega) implements Speeds {
    @Override
    public FieldSpeeds asFieldRelative(Rotation2d robotAngle) {
      return this;
    }

    @Override
    public RobotSpeeds asRobotRelative(Rotation2d robotAngle) {
      return new RobotSpeeds(
          vx * robotAngle.getCos() + vy * robotAngle.getSin(),
          -vx * robotAngle.getSin() + vy * robotAngle.getCos(),
          omega);
    }

    @Override
    public ChassisSpeeds toWpilib() {
      return new ChassisSpeeds(vx, vy, omega);
    }

    public LinearVelocity vxMeasure() {
      return MetersPerSecond.of(vx());
    }

    public LinearVelocity vyMeasure() {
      return MetersPerSecond.of(vy());
    }

    public AngularVelocity omegaMeasure() {
      return RadiansPerSecond.of(omega());
    }

    @Override
    public FieldSpeeds discretize(double dtSeconds) {
      return Internals.discretizePrim(vx, vy, omega, dtSeconds, FieldSpeeds::new);
    }

    public static final FieldSpeeds kZero = new FieldSpeeds(0.0, 0.0, 0.0);

    public static final SpeedsStruct struct = new SpeedsStruct();
  }

  /**
   * Speeds in the robot coordinate system.
   *
   * <ul>
   *   <li>Positive x is forward.
   *   <li>Positive y is left.
   *   <li>Positive omega is counter-clockwise.
   *   <li>(0, 0) is the center of the robot.
   * </ul>
   */
  public record RobotSpeeds(double vx, double vy, double omega) implements Speeds {
    @Override
    public FieldSpeeds asFieldRelative(Rotation2d robotAngle) {
      return new FieldSpeeds(
          vx * robotAngle.getCos() - vy * robotAngle.getSin(),
          vx * robotAngle.getSin() + vy * robotAngle.getCos(),
          omega);
    }

    @Override
    public RobotSpeeds asRobotRelative(Rotation2d robotAngle) {
      return this;
    }

    @Override
    public ChassisSpeeds toWpilib() {
      return new ChassisSpeeds(vx, vy, omega);
    }

    public LinearVelocity vxMeasure() {
      return MetersPerSecond.of(vx());
    }

    public LinearVelocity vyMeasure() {
      return MetersPerSecond.of(vy());
    }

    public AngularVelocity omegaMeasure() {
      return RadiansPerSecond.of(omega());
    }

    @Override
    public RobotSpeeds discretize(double dtSeconds) {
      return Internals.discretizePrim(vx, vy, omega, dtSeconds, RobotSpeeds::new);
    }

    public static final RobotSpeeds kZero = new RobotSpeeds(0.0, 0.0, 0.0);

    public static final SpeedsStruct struct = new SpeedsStruct();
  }

  static FieldSpeeds fromFieldRelative(double vx, double vy, double omega) {
    return new FieldSpeeds(vx, vy, omega);
  }

  static RobotSpeeds fromRobotRelative(double vx, double vy, double omega) {
    return new RobotSpeeds(vx, vy, omega);
  }

  static RobotSpeeds fromRobotRelative(ChassisSpeeds speeds) {
    return new RobotSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  static FieldSpeeds fromFieldRelative(ChassisSpeeds speeds) {
    return new FieldSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  FieldSpeeds asFieldRelative(Rotation2d robotAngle);

  RobotSpeeds asRobotRelative(Rotation2d robotAngle);

  ChassisSpeeds toWpilib();

  Speeds discretize(double dtSeconds);

  static class Internals {

    @FunctionalInterface
    private interface SpeedConstructor<S extends Speeds> {
      S construct(double vx, double vy, double omega);
    }

    private static final <S extends Speeds> S discretizePrim(
        double vx, double vy, double omega, double dt, SpeedConstructor<S> constructor) {
      double cos = Pose2d.kZero.getRotation().getCos();
      double sin = Pose2d.kZero.getRotation().getSin();

      double dx = vx * dt;
      double dy = vy * dt;
      double dtheta = omega * dt;

      double transformSin = Math.sin(dtheta);
      double transformCos = Math.cos(dtheta);

      double s;
      double c;
      if (Math.abs(dtheta) < 1E-9) {
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
        c = 0.5 * dtheta;
      } else {
        s = transformSin / dtheta;
        c = (1 - transformCos) / dtheta;
      }

      double transformX = (dx * s) - (dy * c);
      double transformY = (dx * c) + (dy * s);

      double rotatedX = (transformX * cos) - (transformY * sin);
      double rotatedY = (transformX * sin) + (transformY * cos);

      return constructor.construct(rotatedX / dt, rotatedY / dtheta, omega);
    }
  }

  class SpeedsStruct implements Struct<Speeds> {
    enum SpeedsType implements StructSerializable {
      FIELD,
      ROBOT;

      public static final Struct<SpeedsType> struct =
          ProceduralStructGenerator.genEnum(SpeedsType.class);
    }

    @Override
    public Class<Speeds> getTypeClass() {
      return Speeds.class;
    }

    @Override
    public String getTypeName() {
      return "Speeds";
    }

    @Override
    public int getSize() {
      return SpeedsType.struct.getSize() + 3 * Double.BYTES;
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {SpeedsType.struct};
    }

    @Override
    public boolean isImmutable() {
      return true;
    }

    @Override
    public void pack(ByteBuffer bb, Speeds value) {
      if (value instanceof FieldSpeeds fieldSpeeds) {
        SpeedsType.struct.pack(bb, SpeedsType.FIELD);
        bb.putDouble(fieldSpeeds.vx());
        bb.putDouble(fieldSpeeds.vy());
        bb.putDouble(fieldSpeeds.omega());
      } else if (value instanceof RobotSpeeds robotSpeeds) {
        SpeedsType.struct.pack(bb, SpeedsType.ROBOT);
        bb.putDouble(robotSpeeds.vx());
        bb.putDouble(robotSpeeds.vy());
        bb.putDouble(robotSpeeds.omega());
      } else {
        throw new IllegalArgumentException("Unknown Speeds type");
      }
    }

    @Override
    public Speeds unpack(ByteBuffer bb) {
      SpeedsType type = SpeedsType.struct.unpack(bb);
      switch (type) {
        case FIELD:
          return new FieldSpeeds(bb.getDouble(), bb.getDouble(), bb.getDouble());
        case ROBOT:
          return new RobotSpeeds(bb.getDouble(), bb.getDouble(), bb.getDouble());
        default:
          throw new IllegalArgumentException("Unknown Speeds type");
      }
    }

    @Override
    public String getSchema() {
      return "SpeedsType type; double vx; double vy; double omega;";
    }
  }
}
