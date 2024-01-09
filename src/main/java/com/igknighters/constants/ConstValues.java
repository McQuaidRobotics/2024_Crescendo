package com.igknighters.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.igknighters.ConstantHelper.*;
import com.igknighters.util.SwerveModuleConstants;
import com.igknighters.util.SwerveModuleConstants.ModuleId;
import com.igknighters.vision.Camera;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

//this will be where we put references to all our initialized values
public final class ConstValues {
    // all measurements are in meters unless otherwise specified
    // all angles are in radians unless otherwise specified
    @SuppressWarnings("unused")
    private static final class Conv {
        public static final double FEET_TO_METERS = 0.3048;
        public static final double INCHES_TO_METERS = 0.0254;
        public static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
        public static final double ROTATIONS_TO_RADIANTS = 2 * Math.PI;
    }

    public static final boolean DEBUG = true; // this should be false for competition
    public static final double PERIODIC_TIME = 0.02; // 20ms

    public static final class kDimensions {
        public static final double ROBOT_WIDTH = Units.inchesToMeters(26);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(26);
        public static final double BUMPER_THICKNESS = Units.inchesToMeters(2.7);
    }

    public static final class kVision {
        /** The least trustworthy std dev */
        public static final Vector<N3> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
        /** The middle trustworthy std dev */
        public static final Vector<N3> visionStdDevsTrust = VecBuilder.fill(0.4, 0.4, 0.4);
        /** The most trustworthy std dev */
        public static final Vector<N3> visionStdDevsReal = VecBuilder.fill(0.15, 0.15, 0.15);

        public static final double ToleratedHistoryDifference = 0.1;
        public static final double ToleratedMultiCamDifference = 0.1;


        public static final Camera[] CAMERAS = new Camera[] {
            new Camera(
                "RearLeftCamera",
                0,
                new Pose3d(
                    new Translation3d(0.0, 0.0, 0.25),
                    new Rotation3d()
                )
            ),
            new Camera(
                "RearRightCamera",
                1,
                new Pose3d(
                    new Translation3d(0.0, 0.0, 0.25),
                    new Rotation3d()
                )
            )
        };
    }

    public static final class kSwerve {
        @SuppressWarnings("unused")
        private static final class SwerveGearRatios {
            static final double L1_DRIVE = 1.0 / 8.14;
            static final double L2_DRIVE = 1.0 / 6.75;
            static final double L3_DRIVE = 1.0 / 6.12;
            static final double L4_DRIVE = 1.0 / 5.14;

            static final double ANGLE = 7.0 / 150.0;
        }

        public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2023ChargedUp;

        public static final int PIGEON_ID = 33;
        public static final boolean INVERT_GYRO = false;
        public static final String CANBUS = "DriveBus";

        @DoubleConst(crash = 4.5)
        public static double MAX_DRIVE_VELOCITY;

        @DoubleConst(crash = 10.0)
        public static double MAX_ANGULAR_VELOCITY;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.551942;
        public static final double WHEEL_BASE = 0.551942;
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double DRIVEBASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2));

        public static final double ANGLE_GEAR_RATIO = SwerveGearRatios.ANGLE;

        @DoubleConst(crash = SwerveGearRatios.L2_DRIVE)
        public static double DRIVE_GEAR_RATIO;

        /* Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                Mod0.CHASSIS_OFFSET,
                Mod1.CHASSIS_OFFSET,
                Mod2.CHASSIS_OFFSET,
                Mod3.CHASSIS_OFFSET
            );

        public static final class DriveMotorConstants {
            public static final double kP = 0.25;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final class AngleMotorConstants {
            public static final double kP = 9.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final class Mod0 {
            public static final ModuleId MODULE = ModuleId.m0;
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 21;
            public static final double ROTATION_OFFSET = 0.21875;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET, ROTATION_OFFSET);
        }

        public static final class Mod1 {
            public static final ModuleId MODULE = ModuleId.m1;
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANLGE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 22;
            public static final double ROTATION_OFFSET = 0.3278805;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANLGE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET, ROTATION_OFFSET);
        }

        public static final class Mod2 {
            public static final ModuleId MODULE = ModuleId.m2;
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 23;
            public static final double ROTATION_OFFSET = 0.6540972;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET, ROTATION_OFFSET);
        }

        public static final class Mod3 {
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

    public static final class kAuto {
        public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(3.0, 0.0, 0.0);
        public static final PIDConstants AUTO_ANGULAR_PID = new PIDConstants(3.0, 0.0, 1.0);
    }
}
