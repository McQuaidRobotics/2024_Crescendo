package com.igknighters.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.igknighters.Constants.kSwerve.Mod1;
import com.igknighters.Constants.kSwerve.Mod2;
import com.igknighters.Constants.kSwerve.Mod3;
import com.igknighters.constants.ConstantHelper.*;
import com.igknighters.util.SwerveModuleConstants;
import com.igknighters.util.SwerveModuleConstants.ModuleId;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

//this will be where we put references to all our initialized values
public class ConstValues {
    // all measurements are in meters unless otherwise specified
    // all angles are in radians unless otherwise specified
    @SuppressWarnings("unused")
    private static class Conv {
        public static double FEET_TO_METERS = 0.3048;
        public static double INCHES_TO_METERS = 0.0254;
        public static double DEGREES_TO_RADIANS = Math.PI / 180.0;
        public static double ROTATIONS_TO_RADIANTS = 2 * Math.PI;
    }

    public static boolean DEBUG = true; // this should be false for competition
    public static double PERIODIC_TIME = 0.02; // 20ms

    public static double DEFAULT_DEADBAND = 0.15;

    public static class kDimensions {
        public static double ROBOT_WIDTH = Units.inchesToMeters(26);
        public static double ROBOT_LENGTH = Units.inchesToMeters(26);
        public static double BUMPER_THICKNESS = Units.inchesToMeters(2.7);
    }

    public static class kSwerve {
        @SuppressWarnings("unused")
        private static class SwerveGearRatios {
            public static final double L1_DRIVE = 1.0 / 8.14;
            public static final double L2_DRIVE = 1.0 / 6.75;
            public static final double L3_DRIVE = 1.0 / 6.12;
            public static final double L4_DRIVE = 1.0 / 5.14;

            public static final double ANGLE = 7.0 / 150.0;
        }

        public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2023ChargedUp;

        public static final int PIGEON_ID = 33;
        public static final boolean INVERT_GYRO = false;
        public static final String CANBUS = "DriveBus";

        @DoubleConst(yin = 4.5, yang = 4.5)
        public static double MAX_DRIVE_VELOCITY;

        @DoubleConst(yin = 10.0, yang = 10.0)
        public static double MAX_ANGULAR_VELOCITY;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.551942;
        public static final double WHEEL_BASE = 0.551942;
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double ANGLE_GEAR_RATIO = SwerveGearRatios.ANGLE;

        @DoubleConst(yin = SwerveGearRatios.L2_DRIVE, yang = SwerveGearRatios.L2_DRIVE)
        public static double DRIVE_GEAR_RATIO;

        public static final double METERS_PER_DRIVE_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO;

        /* Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                Mod0.CHASSIS_OFFSET,
                Mod1.CHASSIS_OFFSET,
                Mod2.CHASSIS_OFFSET,
                Mod3.CHASSIS_OFFSET);

        public static class DriveMotorConstants {
            public static double kP = 0.25;
            public static double kI = 0.0;
            public static double kD = 0.0;
        }

        public static class AngleMotorConstants {
            public static double kP = 9.0;
            public static double kI = 0.0;
            public static double kD = 0.0;
        }

        public static class Mod0 {
            public static final ModuleId MODULE = ModuleId.m0;
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 21;
            public static final double ROTATION_OFFSET = 0.21875;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET, ROTATION_OFFSET);
        }

        public static class Mod1 {
            public static final ModuleId MODULE = ModuleId.m1;
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANLGE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 22;
            public static final double ROTATION_OFFSET = 0.3278805;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANLGE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET, ROTATION_OFFSET);
        }

        public static class Mod2 {
            public static final ModuleId MODULE = ModuleId.m2;
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 23;
            public static final double ROTATION_OFFSET = 0.6540972;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET, ROTATION_OFFSET);
        }

        public static class Mod3 {
            public static final ModuleId MODULE = ModuleId.m3;
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 24;
            public static final double ROTATION_OFFSET = 0.5776361;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET, ROTATION_OFFSET);
        }
    }
}
