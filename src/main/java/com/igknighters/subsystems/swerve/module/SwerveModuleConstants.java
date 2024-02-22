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

    private final String moduleIdFieldName = "MODULE";
    private final String driveMotorIDFieldName = "DRIVE_MOTOR_ID";
    private final String angleMotorIDFieldName = "ANGLE_MOTOR_ID";
    private final String cancoderIDFieldName = "CANCODER_ID";
    private final String rotationOffsetFieldName = "ROTATION_OFFSET";
    private final String moduleChassisPoseFieldName = "CHASSIS_OFFSET";

    private Class<?> moduleconsts;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param moduleconsts The class containing the constants for the module
     */
    public SwerveModuleConstants(Class<?> moduleconsts) {
        this.moduleconsts = moduleconsts;
    }

    public Object getConst(String fieldName) {
        try {
            return moduleconsts.getField(fieldName).get(null);
        } catch (Exception e) {
            throw new RuntimeException("Could not get field " + fieldName + " from " + moduleconsts.getName());
        }
    }

    public ModuleId getModuleId() {
        return (ModuleId) getConst(moduleIdFieldName);
    }

    public int getDriveMotorID() {
        return (int) getConst(driveMotorIDFieldName);
    }

    public int getAngleMotorID() {
        return (int) getConst(angleMotorIDFieldName);
    }

    public int getCancoderID() {
        return (int) getConst(cancoderIDFieldName);
    }

    public double getRotationOffset() {
        return (double) getConst(rotationOffsetFieldName);
    }

    public Translation2d getModuleChassisPose() {
        return (Translation2d) getConst(moduleChassisPoseFieldName);
    }
}
