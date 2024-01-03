package com.igknighters.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConstants {
    public static enum ModuleId {
        m0(0),
        m1(1),
        m2(2),
        m3(3);

        public final Integer num;

        ModuleId(Integer num) {
            this.num = num;
        }
    }

    public final ModuleId moduleId;
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Translation2d moduleChassisPose;
    private final double rotationOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param rotationOffset
     */
    public SwerveModuleConstants(ModuleId moduleId, int driveMotorID, int angleMotorID, int canCoderID,
            Translation2d modulePosition, double rotationOffset) {
        this.moduleId = moduleId;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.moduleChassisPose = modulePosition;
        this.rotationOffset = rotationOffset;
    }

    public Rotation2d getRotationOffset(ModuleId module) {
        if (module == ModuleId.m3)
            return Rotation2d.fromDegrees(findCoterminalAngle((rotationOffset * 360) + 225));
        else if (module == ModuleId.m0)
            return Rotation2d.fromDegrees(findCoterminalAngle((rotationOffset * 360) + 315));
        else if (module == ModuleId.m2)
            return Rotation2d.fromDegrees(findCoterminalAngle((rotationOffset * 360) + 125));
        else if (module == ModuleId.m1)
            return Rotation2d.fromDegrees(findCoterminalAngle((rotationOffset * 360) + 45));
        throw new IllegalArgumentException("Module not found");
    }

    private double findCoterminalAngle(double angleOffset) {
        return (angleOffset > 360) ? angleOffset % 360 : angleOffset;
    }
}
