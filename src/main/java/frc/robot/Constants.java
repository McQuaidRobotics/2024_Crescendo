package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.SwerveModuleConstants.ModuleId;

public final class Constants {

    public static class ControllerConsts {
        public static final double LEFT_DEADBAND = 0.15;
        public static final double RIGHT_DEADBAND = 0.15;
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