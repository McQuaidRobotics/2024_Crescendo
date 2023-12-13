package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.SwerveModuleConstants.ModuleId;

public final class Constants {

    public static class ControllerConsts {
        public static final double LEFT_DEADBAND = 0.15;
        public static final double RIGHT_DEADBAND = 0.15;
    }

    public static class kSuperStructure {
        public static final int BRAKE_SWITCH_PIN = 16;
        public static final String CANBUS = "SuperstructureBus";

        public static final class kWrist {
            public static final int MOTOR_ID = 12;
            public static final double MOTOR_kP = 1.0;
            public static final double MOTOR_kI = 0.0;
            public static final double MOTOR_kD = 0.0;

            /** For every {@value} rotations of the motor the mechanism moves 1 rotation */
            // motor -> (10t -> 72t) -> (20t -> 72t) -> (24t -> 48t)
            public static final double MOTOR_TO_MECHANISM_RATIO = (10.0 / 72.0) * (20.0 / 72.0) * (24.0 / 48.0);

            public static final double MAX_VELOCITY = 75;
            public static final double MAX_ACCELERATION = 333;
            public static final double MAX_JERK = 2000;

            public static final boolean INVERTED = false;

            /**
             * Zero is parallel with the elevator
             * <p>
             * Alias for {@link Specs#WRIST_MAX_ANGLE}
             */
            public static final double MAX_DEGREES = Specs.WRIST_MAX_ANGLE;
            /**
             * Zero is parallel with the floor
             * <p>
             * Alias for {@link Specs#WRIST_MIN_ANGLE}
             */
            public static final double MIN_DEGREES = Specs.WRIST_MIN_ANGLE;

            public static final boolean ENABLE_SOFTLIMITS = false;

            /**
             * the ammount of current the motor needs to pull to
             * be recognized at mechanical limit.
             */
            public static final double CURRENT_PEAK_FOR_ZERO = 40.0;

            /** Alias for {@link Specs#WRIST_MAX_ANGLE} */
            public static final double HOME_DEGREES = Specs.WRIST_MAX_ANGLE;
            public static final double HARD_OFFSET = 3.5;

            public static final double TOLERANCE = 0.5;
        }

        public static final class kIntake {
            public static final int MOTOR_ID = 11;

            public static final boolean INVERTED = true;
            public static final boolean BREAK_DEFAULT = true;

            public static final double CURRENT_LIMIT = 15.0;
        }

        public static final class kPivot {
            public static final int LEFT_MOTOR_ID = 13;
            public static final int RIGHT_MOTOR_ID = 14;
            public static final int PIGEON_ID = 31;

            public static final double MOTOR_kP = 1.0;
            public static final double MOTOR_kI = 0;
            public static final double MOTOR_kD = 0;

            public static final double MAX_VELOCITY = 105;
            public static final double MAX_ACCELERATION = 700;
            public static final double MAX_JERK = 10000;// effectively infinite

            /**
             * Zero is parallel with the floor
             * <p>
             * Alias for {@link Specs#PIVOT_MIN_ANGLE}
             */
            public static final double MIN_DEGREES = Specs.PIVOT_MIN_ANGLE;
            /**
             * Zero is parallel with the floor
             * <p>
             * Alias for {@link Specs#PIVOT_MAX_ANGLE}
             */
            public static final double MAX_DEGREES = Specs.PIVOT_MAX_ANGLE;

            public static final double HOME_DEGREES = Specs.PIVOT_MIN_ANGLE + 2.0;
            public static final double PIGEON_OFFSET = 1.85;

            public final static double SCORE_DEGREES = 40.0;

            public static final boolean ENABLE_SOFTLIMITS = false;

            /** For every {@value} rotations of the motor the mechanism moves 1 rotation */
            // motor -> gbx(25:1) -> (30t -> 64t) -> (12t -> 54t)
            public static final double MOTOR_TO_MECHANISM_RATIO = (1.0 / 25.0) * (30.0 / 64.0) * (12.0 / 54.0);

            public static final boolean INVERTED = false;

            /**
             * The max voltage of the motors to behave more predictably
             * throughout the match.
             */
            public static final double VOLTAGE_COMP = 11.8;

            /**
             * The ammount of current the motor needs to pull to
             * be recognized at mechanical limit.
             */
            public static final double CURRENT_PEAK_FOR_HOME = 35.0;

            public static final double TOLERANCE = 0.5;
        }

        public static final class kElevator {
            public static final int ELEVATOR_LEFT_MOTOR_ID = 17;
            public static final int ELEVATOR_RIGHT_MOTOR_ID = 16;

            public static final double MOTOR_kP = 0.5;
            public static final double MOTOR_kD = 0.0;
            public static final double MOTOR_kI = 0.0;

            public static final boolean ENABLE_SOFTLIMITS = false;

            public static final boolean INVERTED = false;

            public static final double MAX_VELOCITY = 50;
            public static final double MAX_ACCELERATION = 250;
            public static final double MAX_JERK = 625;

            public static final double MOTOR_TO_MECHANISM_RATIO = 1.0 / 3.0;
            public static final double MECHANISM_DIAMETER_METERS = 0.0425; // approximate

            /** Alias for {@link Specs#ELEVATOR_MIN_METERS} */
            public static final double HOME_METERS = Specs.ELEVATOR_MIN_METERS;
            public static final double HARD_OFFSET = 0.1;

            /** Alias for {@link Specs#ELEVATOR_MIN_METERS} */
            public static final double MIN_METERS = Specs.ELEVATOR_MIN_METERS;
            /** Alias for {@link Specs#ELEVATOR_MAX_METERS} */
            public static final double MAX_METERS = Specs.ELEVATOR_MAX_METERS;

            public static final double TOLERANCE = 0.075;
        }

        public static final class Specs {
            public static final double WRIST_MASS_GRAMS = 3250;
            public static final double ARM_MASS_GRAMS = 9500;

            public static final double ELEVATOR_MIN_METERS = 0.565;
            public static final double ELEVATOR_MAX_METERS = 1.5;

            public static final double PIVOT_MIN_ANGLE = -8.0;
            public static final double PIVOT_MAX_ANGLE = 90.0;

            // arbitrary min, not mechanical limit(which is ~15 less)
            public static final double WRIST_MIN_ANGLE = -66.04;
            public static final double WRIST_MIN_ANGLE_FLOOR = 21.1;
            public static final double WRIST_MAX_ANGLE = 149.39;

            public static final Transform3d PIVOT_OFFSET_METERS = new Transform3d(
                    new Translation3d(0.0, -0.232953, 0.252125),
                    new Rotation3d());

            public static final double RIM_ABOVE_FLOOR_METERS = 0.09525;
        }

    }

    public static class kAuto {
        public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(3.5, 0.0, 0.0);
        public static final PIDConstants AUTO_ANGULAR_PID = new PIDConstants(3.0, 0.0, 0.0);
    }

    public static class kSwerve {
        public static final int PIGEON_ID = 33;
        public static final boolean INVERT_GYRO = false;
        public static final String CANBUS = "DriveBus";

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.551942;
        public static final double WHEEL_BASE = 0.551942;
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /** For every {@value} rotations of the motor the wheel rolls one rotation */
        public static final double DRIVE_MECHANISM_RATIO = 1.0 / 6.75;
        /** For every {@value} rotations of the motor the wheel spins one rotation */
        public static final double ANGLE_MECHANISM_RATIO = 7.0 / 150.0;
        public static final double METERS_PER_DRIVE_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE * DRIVE_MECHANISM_RATIO;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // idk why this is needed?
                Mod1.CHASSIS_OFFSET,
                Mod2.CHASSIS_OFFSET,
                Mod3.CHASSIS_OFFSET);

        /* Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 4.0;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.25;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final class Sim {
            // Volts to meters/sec
            public static final double DRIVE_KV = 3.42;
            // Volts to meters/sec^2
            public static final double DRIVE_KA = 0.265;

            // Volts to deg/sec
            public static final double ROTATION_KV = 12.0 / 2.5;
            // Volts to deg/sec^2
            public static final double ROTATION_KA = 0.00004;
        }

        public static final class Mod0 {
            public static final ModuleId MODULE = ModuleId.m0;
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 21;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET);
        }

        public static final class Mod1 {
            public static final ModuleId MODULE = ModuleId.m1;
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANLGE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 22;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANLGE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET);
        }

        public static final class Mod2 {
            public static final ModuleId MODULE = ModuleId.m2;
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 23;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET);
        }

        public static final class Mod3 {
            public static final ModuleId MODULE = ModuleId.m3;
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 24;
            public static final Translation2d CHASSIS_OFFSET = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(MODULE, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, CHASSIS_OFFSET);
        }
    }
}