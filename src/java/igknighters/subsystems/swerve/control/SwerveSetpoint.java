package igknighters.subsystems.swerve.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.subsystems.swerve.module.SwerveModule.AdvancedSwerveModuleState;
import monologue.ProceduralStructGenerator;
import monologue.ProceduralStructGenerator.FixedSizeArray;
import wpilibExt.Speeds.RobotSpeeds;

public record SwerveSetpoint(
    RobotSpeeds speeds, @FixedSizeArray(size = 4) AdvancedSwerveModuleState[] moduleStates)
    implements StructSerializable {
  public static SwerveSetpoint zeroed() {
    return new SwerveSetpoint(
        RobotSpeeds.kZero,
        new AdvancedSwerveModuleState[] {
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0)
        });
  }

  public static final Struct<SwerveSetpoint> struct =
      ProceduralStructGenerator.genRecord(SwerveSetpoint.class);
}
