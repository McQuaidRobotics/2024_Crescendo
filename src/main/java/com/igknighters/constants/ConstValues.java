package com.igknighters.constants;

import com.igknighters.constants.ConstantHelper.*;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

//this will be where we put references to all our initialized values
public class ConstValues {
    // all measurements are in meters unless otherwise specified
    // all angles are in radians unless otherwise specified
    @SuppressWarnings("unused")
    private static class Conv {
        public static final double FEET_TO_METERS = 0.3048;
        public static final double INCHES_TO_METERS = 0.0254;
        public static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
        public static final double ROTATIONS_TO_RADIANTS = 2 * Math.PI;
    }

    public static final boolean DEBUG = true; // this should be false for competition
    public static final double PERIODIC_TIME = 0.02; // 20ms

    public static double DEFAULT_DEADBAND = 0.15;

    public static class kDimensions {
        public static double ROBOT_WIDTH = Units.inchesToMeters(26);
        public static double ROBOT_LENGTH = Units.inchesToMeters(26);
        public static double BUMPER_THICKNESS = Units.inchesToMeters(2.7);
    }

    public static class kSwerve {
        @SuppressWarnings("unused")
        private static class SwerveGearRatios {
            public static final double L1_DRIVE = 8.14;
            public static final double L2_DRIVE = 6.75;
            public static final double L3_DRIVE = 6.12;
            public static final double L4_DRIVE = 5.14;

            public static final double ANGLE = 150d / 7d;
        }

        public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2023ChargedUp;

        public static boolean PREFER_X_ORIENTED_PATHS = true;
        public static String CANIVORE_NAME = "McQDriveBus";

        @DoubleConst(yin = 16d * Conv.FEET_TO_METERS, yang = 18d * Conv.FEET_TO_METERS)
        public static double MAX_DRIVE_VELOCITY;
        public static double MAX_DRIVE_ACCELERATION = 2.5;
        public static double MAX_DRIVE_JERK = 60d;

        public static double MAX_TURN_VELOCITY = 10d;
        public static double MAX_TURN_ACCELERATION = 14d;
        public static double MAX_TURN_JERK = 60d;

        public static double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
        public static double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);

        public static double SLIP_CURRENT_CAP = 20;

        public static int GYRO_ID = 40;

        public static double WHEEL_DIAMETER = Units.inchesToMeters(4);

        public static double ANGLE_GEAR_RATIO = SwerveGearRatios.ANGLE;

        @DoubleConst(yin = SwerveGearRatios.L2_DRIVE, yang = SwerveGearRatios.L3_DRIVE)
        public static double DRIVE_GEAR_RATIO;

        public static boolean INVERT_ANGLE_MOTORS = true;

        public static class DriveMotorConstants {
            public static double kP = 0.0001;
            public static double kI = 0.0;
            public static double kD = 0.0;
            public static double kS = 0.67468;
            public static double kV = 2.14939;
            public static double kA = 0.0001;
        }

        public static class AngleMotorConstants {
            public static double kP = 0.2;
            public static double kI = 0.0;
            public static double kD = 1.0;
            public static double kS = 0.55;
            public static double kV = 0.23;
            public static double kA = 0.0056;
        }

        public static class kFrontLeft {
            public static int ENCODER_ID = 22;
            public static int DRIVE_MOTOR_ID = 3;
            public static int ANGLE_MOTOR_ID = 4;
            public static double ENCODER_OFFSET = 85.78;
        }

        public static class kFrontRight {
            public static int ENCODER_ID = 21;
            public static int DRIVE_MOTOR_ID = 1;
            public static int ANGLE_MOTOR_ID = 2;
            public static double ENCODER_OFFSET = 32.6;
        }

        public static class kBackLeft {
            public static int ENCODER_ID = 23;
            public static int DRIVE_MOTOR_ID = 5;
            public static int ANGLE_MOTOR_ID = 6;
            public static double ENCODER_OFFSET = 343.74;
        }

        public static class kBackRight {
            public static int ENCODER_ID = 24;
            public static int DRIVE_MOTOR_ID = 7;
            public static int ANGLE_MOTOR_ID = 8;
            public static double ENCODER_OFFSET = 163.56;
        }
    }
}
