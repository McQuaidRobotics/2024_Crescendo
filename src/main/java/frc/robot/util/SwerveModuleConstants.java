package frc.robot.util;

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

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(ModuleId moduleId, int driveMotorID, int angleMotorID, int canCoderID,
            Translation2d modulePosition) {
        this.moduleId = moduleId;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.moduleChassisPose = modulePosition;
    }
}
