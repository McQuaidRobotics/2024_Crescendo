package com.igknighters.subsystems.swerve.module;

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
    public final double rotationOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorID The ID of the drive motor
     * @param angleMotorID The ID of the angle motor
     * @param canCoderID  The ID of the cancoder
     * @param rotationOffset The rotation offset of the cancoder
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
}
