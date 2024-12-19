package igknighters.subsystems.swerve.control;

import igknighters.subsystems.swerve.module.SwerveModule.AdvancedSwerveModuleState;
import monologue.procstruct.ProceduralStructGenerator;
import monologue.procstruct.ProceduralStructGenerator.FixedSizeArray;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public record SwerveSetpoint(
    ChassisSpeeds chassisSpeeds,
    @FixedSizeArray( size = 4 )
    AdvancedSwerveModuleState[] moduleStates
) implements StructSerializable {
    public static SwerveSetpoint zeroed() {
        return new SwerveSetpoint(new ChassisSpeeds(), new AdvancedSwerveModuleState[] {
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0)
        });
    }

    public static final Struct<SwerveSetpoint> struct;
    static {
        struct = ProceduralStructGenerator.genRecord(SwerveSetpoint.class);
    }
}
