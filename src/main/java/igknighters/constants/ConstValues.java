package igknighters.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import igknighters.constants.ConstValues.kStem.kTelescope;
import igknighters.constants.RobotConfig.RobotID;
import igknighters.subsystems.stem.StemSolvers.AimSolveStrategy;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class ConstValues {
    private final static double TAU = 2 * Math.PI;

    // all measurements are in meters unless otherwise specified
    // all angles are in radians unless otherwise specified
    static final class Conv {
        public static final double FEET_TO_METERS = 0.3048;
        public static final double INCHES_TO_METERS = 0.0254;
        public static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
        public static final double ROTATIONS_TO_RADIANTS = TAU;
        public static final double RPM_TO_RADIANS_PER_SECOND = TAU / 60.0;
    }

    @SuppressWarnings("unused")
    private static final class Motors {
        private static final class Falcon500 {
            public static final double FREE_SPEED = 668.1;
            public static final double FREE_CURRENT = 1.5;
            public static final double STALL_TORQUE = 4.69;
            public static final double STALL_CURRENT = 257.0;
        }

        private static final class Falcon500Foc {
            public static final double FREE_SPEED = 636.7;
            public static final double FREE_CURRENT = 1.5;
            public static final double STALL_TORQUE = 5.84;
            public static final double STALL_CURRENT = 304.0;
        }

        public static final class KrakenX60Foc {
            public static final double FREE_SPEED = 608.0;
            public static final double FREE_CURRENT = 2.0;
            public static final double STALL_TORQUE = 9.37;
            public static final double STALL_CURRENT = 483.0;
        }
    }

    public static final boolean DEBUG = true; // this should be false for competition
    public static final boolean DEMO = false; // this should be false for competition
    public static final boolean SUNLIGHT = false; // this should be false for competition
    public static final double PERIODIC_TIME = 0.02; // 20ms
    public static final int PDH_CAN_ID = 61;

    public static final class kCharacterization {
        public static final int FREQUENCY = 750;

        public static enum MechanismId {
            PIVOT("Pivot"),
            WRIST("Wrist"),
            TELESCOPE("Telescope"),
            SHOOTER("Shooter"),
            SWERVE_ROTATION("Swerve/Rotation"),
            SWERVE_VELOCITY("Swerve/Velocity"),
            SWERVE_STEER("Swerve/Steer");

            public final String name;

            private MechanismId(String name) {
                this.name = name;
            }
        }
    }

    public static final class kRobotCollisionGeometry {
        public static final double BUMPER_THICKNESS = 2.8 * Conv.INCHES_TO_METERS;
        public static final double BUMPER_HEIGHT = 5.75 * Conv.INCHES_TO_METERS;
        public static final double FRAME_WIDTH = 26.0 * Conv.INCHES_TO_METERS;

        public static final double UMBRELLA_LENGTH = 13.25 * Conv.INCHES_TO_METERS;
        public static final double UMBRELLA_HEIGHT = 5.0 * Conv.INCHES_TO_METERS;
        public static final double UMBRELLA_OFFSET = 2.45 * Conv.INCHES_TO_METERS;

        public static final double EXTENSION_MAX = 14.0;

        public static final Rectangle2d DRIVE_BASE = new Rectangle2d(
                new Pose2d(),
                FRAME_WIDTH + (BUMPER_THICKNESS * 2),
                BUMPER_HEIGHT);

        public static final Rectangle2d BOUNDS = new Rectangle2d(
                new Pose2d(new Translation2d(
                        (-EXTENSION_MAX * Conv.INCHES_TO_METERS) + BUMPER_THICKNESS,
                        0.0), Rotation2d.kZero),
                FRAME_WIDTH + ((EXTENSION_MAX * 2) * Conv.INCHES_TO_METERS),
                48.0 * Conv.INCHES_TO_METERS);

        public static final Translation2d PIVOT_LOCATION = new Translation2d(
                ((32.6 / 2.0) - 9.5) * Conv.INCHES_TO_METERS,
                7.25 * Conv.INCHES_TO_METERS);
    }

    public static final class kControls {
        public static final double SHOOTER_RPM = 7000.0;
        public static final double SHOOTER_PASS_RPM = 3900.0;
        public static final double AUTO_SHOOTER_RPM = 8000.0;
        public static final double SHOOTER_IDLE_RPM = 4000.0;
        public static final double INTAKE_PERCENT = 0.8;

        public static final double STATIONARY_AIM_AT_PIVOT_RADIANS = 40.0 * Conv.DEGREES_TO_RADIANS;
        public static final double STATIONARY_PASS_PIVOT_RADIANS = 60.0 * Conv.DEGREES_TO_RADIANS;
        public static final double STATIONARY_PASS_TELESCOPE_METERS = kTelescope.MIN_METERS
                + (0.0 * Conv.INCHES_TO_METERS);
        public static final double STATIONARY_WRIST_ANGLE = 71.0 * Conv.DEGREES_TO_RADIANS;
        public static final double MAX_HEIGHT_AIM_AT_PIVOT_RADIANS = 86.0 * Conv.DEGREES_TO_RADIANS;
        public static final double MAX_HEIGHT_AIM_AT_TELESCOPE_METERS = kTelescope.MAX_METERS;

        public static final AimSolveStrategy DEFAULT_AIM_STRATEGY = AimSolveStrategy.STATIONARY_PIVOT_GRAVITY;

        public static final double SOTM_LOOKAHEAD_TIME = 0.275;

        public static final Translation2d PASS_LAND_LOCATION = new Translation2d(
                1.0, 7.0);

        public static final Translation2d PASS_SHOOT_FROM_LOCATION = new Translation2d(
                10.0, 0.7);
    }

    public static final class kLed {
        public static final int LED_COUNT = 38;
        public static final int CANDLE_LEDS = 8;
    }

    public static final class kVision {
        public static final double ROOT_TRUST = 0.75;

        public static final double MAX_Z_DELTA = 0.2;
        public static final double MAX_ANGLE_DELTA = 5.0 * Conv.DEGREES_TO_RADIANS;

        public static enum CameraConfigs {
            CRASH(
                    new CameraConfig[] {
                            new CameraConfig(
                                    "photon_module_1",
                                    new Transform3d(
                                            new Translation3d(
                                                    -11.3 * Conv.INCHES_TO_METERS,
                                                    -8.6 * Conv.INCHES_TO_METERS,
                                                    8.0 * Conv.INCHES_TO_METERS),
                                            new Rotation3d(
                                                    0.0,
                                                    -20.0 * Conv.DEGREES_TO_RADIANS,
                                                    Math.PI))),
                            new CameraConfig(
                                    "photon_module_2",
                                    new Transform3d(
                                            new Translation3d(
                                                    -11.3 * Conv.INCHES_TO_METERS,
                                                    8.6 * Conv.INCHES_TO_METERS,
                                                    8.0 * Conv.INCHES_TO_METERS),
                                            new Rotation3d(
                                                    0.0,
                                                    -20.0 * Conv.DEGREES_TO_RADIANS,
                                                    Math.PI)))
                    }),
            BURN(new CameraConfig[] {});

            public final CameraConfig[] cameras;

            private CameraConfigs(CameraConfig[] cameras) {
                this.cameras = cameras;
            }

            public static CameraConfig[] forRobot(RobotID id) {
                return switch (RobotConfig.getRobotID()) {
                    case CRASH -> CameraConfigs.CRASH.cameras;
                    case SIM_CRASH -> CameraConfigs.CRASH.cameras;
                    default -> CameraConfigs.BURN.cameras;
                };
            }
        }
    }

    public static final class kSwerve {
        /**
         * The gear ratios for the swerve modules for easier constant definition.
         */
        @SuppressWarnings("unused")
        private static final class SwerveGearRatios {
            static final double L1_DRIVE = 8.14;
            static final double L2_DRIVE = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            static final double L3_DRIVE = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
            static final double L3_DRIVE_KRAKEN = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
            static final double L4_DRIVE = 5.14;

            static final double STEER = 150.0 / 7.0;
        }

        public static final int PIGEON_ID = 33;
        public static final boolean INVERT_GYRO = false;
        public static final String CANBUS = "DriveBus";

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.551942;
        public static final double WHEEL_RADIUS = 2.0 * Conv.INCHES_TO_METERS;
        public static final double WHEEL_DIAMETER = WHEEL_RADIUS * 2.0;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        // public static final double DRIVEBASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH
        // / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2));
        public static final double DRIVEBASE_RADIUS = 0.39;
        public static final double DRIVEBASE_CIRCUMFERENCE = DRIVEBASE_RADIUS * TAU;

        public static final double STEER_GEAR_RATIO = SwerveGearRatios.STEER;

        public static final double DRIVE_GEAR_RATIO = SwerveGearRatios.L3_DRIVE_KRAKEN;

        public static final double WHEEL_COF = 1.5;

        /**
         * Not every motor can output the max speed at all times, add a buffer to make
         * closed loop more accurate
         */
        public static final double MOTOR_CLOSED_LOOP_OUTPUT_SCALAR = 0.95;

        /** User defined acceleration time in seconds */
        public static final double ACCELERATION_TIME = 0.7;

        public static final double SLIP_CURRENT = 65.0;

        public static final double MAX_DRIVE_VELOCITY = ((Motors.KrakenX60Foc.FREE_SPEED / TAU) / DRIVE_GEAR_RATIO)
                * WHEEL_CIRCUMFERENCE * MOTOR_CLOSED_LOOP_OUTPUT_SCALAR;

        public static final double MAX_ANGULAR_VELOCITY = MAX_DRIVE_VELOCITY / DRIVEBASE_RADIUS;
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY / ACCELERATION_TIME;

        public static final double MAX_STEERING_VELOCITY = Motors.Falcon500Foc.FREE_SPEED
                / (STEER_GEAR_RATIO * MOTOR_CLOSED_LOOP_OUTPUT_SCALAR);

        /* Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final class kDriveMotor {
            public static final double kP = 0.27;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.15;
            public static final double kV = 12.0
                    / (kSwerve.MAX_DRIVE_VELOCITY / (kSwerve.WHEEL_CIRCUMFERENCE / kSwerve.DRIVE_GEAR_RATIO));
        }

        public static final class kSteerMotor {
            public static final double kP = 11.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final class kRotationController {
            public static final double kP = 9.0;
            public static final double kD = 0.2;

            public static final double CONSTRAINT_SCALAR = 1.0;
        }

        public static final boolean ORIENT_TELEOP_FOR_SIM_DEFAULT = false;

        public static final LerpTable TELEOP_TRANSLATION_AXIS_CURVE = new LerpTable(
                new LerpTableEntry(0.0, 0.0),
                new LerpTableEntry(0.15, 0.0), // deadzone
                new LerpTableEntry(0.7, 0.4),
                new LerpTableEntry(1.0, 1.0));

        public static final LerpTable TELEOP_ROTATION_AXIS_CURVE = new LerpTable(
                new LerpTableEntry(0.0, 0.0),
                new LerpTableEntry(0.15, 0.0), // deadzone
                new LerpTableEntry(0.5, 0.2),
                new LerpTableEntry(0.7, 0.4),
                new LerpTableEntry(1.0, 1.0));

        public static final double[] CRASH_ROTATION_OFFSETS = new double[] {
                -0.1015,
                0.42529,
                -0.4182,
                -0.1086
        };

        public static final double[] BURN_ROTATION_OFFSETS = new double[] {
                0.0824,
                0.10595,
                -0.21533,
                -0.398925
        };

        public static final Translation2d[] MODULE_CHASSIS_OFFSETS = new Translation2d[] {
                new Translation2d(TRACK_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-TRACK_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-TRACK_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(TRACK_WIDTH / 2.0, TRACK_WIDTH / 2.0)
        };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                MODULE_CHASSIS_OFFSETS);
    }

    public static final class kAuto {
        public static final class kTranslation {
            public static final double kP = 3.4;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final class kRotation {
            public static final double kP = 3.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final double AUTO_SHOOTER_RPM = 6000.0;
    }

    public static final class kUmbrella {
        public static final double NOTE_VELO = 23.5;
        public static final String CANBUS = "SuperStructureBus";

        public static final class kShooter {
            public static final double MOTOR_RIGHT_kP = 0.15;
            public static final double MOTOR_RIGHT_kI = 0.0;
            public static final double MOTOR_RIGHT_kD = 0.00;
            public static final double MOTOR_RIGHT_kS = 0.155;
            public static final double MOTOR_RIGHT_kV = 0.118;

            public static final double MOTOR_LEFT_kP = 0.15;
            public static final double MOTOR_LEFT_kI = 0.0;
            public static final double MOTOR_LEFT_kD = 0.00;
            public static final double MOTOR_LEFT_kS = 0.155;
            public static final double MOTOR_LEFT_kV = 0.118;

            public static final int LEFT_MOTOR_ID = 17;
            public static final int RIGHT_MOTOR_ID = 18;

            public static final double MECHANISM_RATIO = 1.5;
            public static final double WHEEL_DIAMETER = 4.0;

            public static final double DEFAULT_TOLERANCE = 0.03;

            public static final double PEAK_CURRENT = 80.0;
            public static final double MAX_SHOOT_SPEED = 8000.0 * Conv.RPM_TO_RADIANS_PER_SECOND;

            public static final double LEFT_MOTOR_DIFF = 0.9;
        }

        public static final class kIntake {
            public static final int UPPER_MOTOR_ID = 19;
            public static final int LOWER_MOTOR_ID = 20;

            public static final double UPPER_DIAMETER = 2.0625 * Conv.INCHES_TO_METERS;
            public static final double LOWER_DIAMETER = 1.25 * Conv.INCHES_TO_METERS;

            public static final double UPPER_MECHANISM_RATIO = 2.0;
            public static final double LOWER_MECHANISM_RATIO = 22.0 / 12.0;

            public static final double UPPER_DIFF = (LOWER_DIAMETER / UPPER_DIAMETER)
                    * (UPPER_MECHANISM_RATIO / LOWER_MECHANISM_RATIO);

            public static final boolean BEAM_IS_UPPER = true;
        }

    }

    public static final class kStem {
        public static final String CANBUS = "SuperStructureBus";

        public static final int COAST_SWITCH_CHANNEL = 9;

        public static final double MECHANICALLY_VIABLE_BUFFER = 0.03;

        public static final class kPivot {

            public static final class kDimensions {
                /**
                 * From the center of the robot
                 */
                public static final Translation2d PIVOT_AXEL_LOCATION = new Translation2d(
                        -9.0 * Conv.INCHES_TO_METERS,
                        7.25 * Conv.INCHES_TO_METERS);
            }

            public static final int LEFT_MOTOR_ID = 11;
            public static final int RIGHT_MOTOR_ID = 12;
            public static final int PIGEON_ID = 31;

            public static final double MOTOR_kP = 0.7;
            public static final double MOTOR_kI = 0.0;
            public static final double MOTOR_kD = 0.0;
            public static final double MOTOR_kS = 0.0;
            public static final double MOTOR_kV = 0.0;

            public static final double MAX_VELOCITY = 100;
            public static final double MAX_ACCELERATION = 4500;
            public static final double MAX_JERK = 0;

            public static final double MIN_ANGLE = 7.0 * Conv.DEGREES_TO_RADIANS;

            public static final double MAX_ANGLE = 97.0 * Conv.DEGREES_TO_RADIANS;

            public static final boolean ENABLE_SOFTLIMITS = true;

            /** For every {@value} rotations of the motor the mechanism moves 1 rotation */
            // motor -> gbx(100:1) -> (15 -> 42) -> mechanism
            public static final double MOTOR_TO_MECHANISM_RATIO = (5.0 * 5.0 * 5.0) * (42.0 / 15.0);

            public static final boolean INVERTED = false;

            /**
             * The max voltage of the motors to behave more predictably
             * throughout the match.
             */
            public static final double VOLTAGE_COMP = 11.0;
            public static final double TARGET_TOLERANCE = 0.75 * Conv.DEGREES_TO_RADIANS;
        }

        public static final class kTelescope {
            public static final int MOTOR_ID = 15;

            public static final double MOTOR_kP = 5.0;
            public static final double MOTOR_kI = 0.0;
            public static final double MOTOR_kD = 0.1;

            public static final double MAX_VELOCITY = 50;
            public static final double MAX_ACCELERATION = 600;
            public static final double MAX_JERK = 3000;

            public static final double MOTOR_TO_MECHANISM_RATIO = 20.0;

            public static final double SPROCKET_CIRCUMFERENCE = 0.895 * TAU * Conv.INCHES_TO_METERS;

            public static final double MIN_METERS = 16.0 * Conv.INCHES_TO_METERS;
            public static final double MAX_METERS = MIN_METERS
                    + ((40.427 / MOTOR_TO_MECHANISM_RATIO) * SPROCKET_CIRCUMFERENCE);

            public static final boolean INVERTED = false;

            /**
             * Tolerance in meters
             */
            public static final double TARGET_TOLERANCE = 1.0 * Conv.INCHES_TO_METERS;
        }

        public static final class kWrist {

            public static final class kDimensions {
                public static final double ANGLE_OFFSET = 38.65 * Conv.DEGREES_TO_RADIANS;
                public static final double MOTOR_PIVOT_TO_WRIST_PIVOT = 3.393 * Conv.INCHES_TO_METERS;
                public static final double WRIST_PIVOT_TO_NUT = 1.566 * Conv.INCHES_TO_METERS;
            }

            public static final int MOTOR_ID = 16;
            public static final int CANCODER_ID = 26;

            public static final double MOTOR_kP = 150.0;
            public static final double MOTOR_kI = 0.0;
            public static final double MOTOR_kD = 3.0;
            public static final double MOTOR_kS = 0.06;
            public static final double MOTOR_kV = 0.0;

            public static final boolean INVERTED = false;

            public static final double CANCODER_OFFSET = -0.6822;

            public static final double MIN_ANGLE = 8.0 * Conv.DEGREES_TO_RADIANS;
            public static final double MAX_ANGLE = 117.0 * Conv.DEGREES_TO_RADIANS;

            public static final double MOTOR_TO_MECHANISM_RATIO = 9.0 * 4.0 * (84.0 / 22.0);

            public static final double MAX_VELOCITY = 1;
            public static final double MAX_ACCELERATION = 10;
            public static final double MAX_JERK = 20;

            /**
             * Tolerance in radians
             */
            public static final double TARGET_TOLERANCE = 1.5 * Conv.DEGREES_TO_RADIANS;

        }
    }
}
