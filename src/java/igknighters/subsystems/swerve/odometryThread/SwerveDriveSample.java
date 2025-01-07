package igknighters.subsystems.swerve.odometryThread;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record SwerveDriveSample(
    SwerveModulePosition[] modulePositions,
    Rotation2d gyroYaw,
    double acceleration,
    double timestamp)
    implements StructSerializable, Cloneable {

  public SwerveDriveSample clone() {
    final SwerveModulePosition[] newModulePositions =
        new SwerveModulePosition[this.modulePositions.length];
    for (int i = 0; i < modulePositions.length; i++) {
      final SwerveModulePosition oldModulePosition = this.modulePositions[i];
      newModulePositions[i] =
          new SwerveModulePosition(oldModulePosition.distanceMeters, oldModulePosition.angle);
    }
    return new SwerveDriveSample(
        newModulePositions,
        new Rotation2d(gyroYaw.getCos(), gyroYaw.getSin()),
        acceleration,
        timestamp);
  }

  public static final Struct<SwerveDriveSample> struct = new SwerveDriveSampleStruct();

  public static class SwerveDriveSampleStruct implements Struct<SwerveDriveSample> {
    @Override
    public Class<SwerveDriveSample> getTypeClass() {
      return SwerveDriveSample.class;
    }

    @Override
    public int getSize() {
      return SwerveModulePosition.struct.getSize() * 4
          + Rotation2d.struct.getSize()
          + Double.BYTES * 2;
    }

    @Override
    public String getTypeName() {
      return "SwerveDriveSample";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {SwerveModulePosition.struct, Rotation2d.struct};
    }

    @Override
    public boolean isImmutable() {
      return true;
    }

    @Override
    public SwerveDriveSample clone(SwerveDriveSample obj) throws CloneNotSupportedException {
      return obj.clone();
    }

    @Override
    public boolean isCloneable() {
      return true;
    }

    @Override
    public String getSchema() {
      return "Rotation2d gyroYaw;"
          + "double gforce;"
          + "double timestamp;"
          + "SwerveModulePosition module0;"
          + "SwerveModulePosition module1;"
          + "SwerveModulePosition module2;"
          + "SwerveModulePosition module3;";
    }

    @Override
    public void pack(ByteBuffer bb, SwerveDriveSample value) {
      Rotation2d.struct.pack(bb, value.gyroYaw);
      bb.putDouble(value.acceleration);
      bb.putDouble(value.timestamp);
      for (SwerveModulePosition module : value.modulePositions) {
        SwerveModulePosition.struct.pack(bb, module);
      }
    }

    @Override
    public SwerveDriveSample unpack(ByteBuffer bb) {
      final Rotation2d gyroYaw = Rotation2d.struct.unpack(bb);
      final double gforce = bb.getDouble();
      final double timestamp = bb.getDouble();
      final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        modulePositions[i] = SwerveModulePosition.struct.unpack(bb);
      }
      return new SwerveDriveSample(modulePositions, gyroYaw, gforce, timestamp);
    }
  }
}
