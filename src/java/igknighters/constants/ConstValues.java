package igknighters.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import igknighters.constants.RobotConfig.RobotID;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;

public final class ConstValues {
  private static final double TAU = 2 * Math.PI;

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
  public static final double PERIODIC_TIME = 0.02; // 20ms
  public static final int PDH_CAN_ID = 61;

  public static final class kCharacterization {
    public static final int FREQUENCY = 750;

    public static enum MechanismId {
      SWERVE_ROTATION("Swerve/Rotation"),
      SWERVE_VELOCITY("Swerve/Velocity"),
      SWERVE_STEER("Swerve/Steer");

      public final String name;

      private MechanismId(String name) {
        this.name = name;
      }
    }
  }

  public static final class kControls {
    public static final double SOTM_LOOKAHEAD_TIME = 0.275;

    public static final Translation2d PASS_LAND_LOCATION = new Translation2d(1.0, 7.0);

    public static final Translation2d PASS_SHOOT_FROM_LOCATION = new Translation2d(10.0, 0.7);
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
                    new Rotation3d(0.0, -20.0 * Conv.DEGREES_TO_RADIANS, Math.PI))),
            new CameraConfig(
                "photon_module_2",
                new Transform3d(
                    new Translation3d(
                        -11.3 * Conv.INCHES_TO_METERS,
                        8.6 * Conv.INCHES_TO_METERS,
                        8.0 * Conv.INCHES_TO_METERS),
                    new Rotation3d(0.0, -20.0 * Conv.DEGREES_TO_RADIANS, Math.PI)))
          }),
      BURN(new CameraConfig[] {});

      public final CameraConfig[] cameras;

      private CameraConfigs(CameraConfig[] cameras) {
        this.cameras = cameras;
      }

      public static CameraConfig[] forRobot(RobotID id) {
        return switch (RobotConfig.getRobotID()) {
          case CRASH -> CameraConfigs.CRASH.cameras;
          default -> CameraConfigs.BURN.cameras;
        };
      }
    }
  }

  public static final class kSwerve {
    /** The gear ratios for the swerve modules for easier constant definition. */
    @SuppressWarnings("unused")
    private static final class SwerveGearRatios {
      static final double L1_DRIVE = 8.14;
      static final double L2_DRIVE = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
      static final double L2_DRIVE_KRAKEN = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
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

    public static final double DRIVE_GEAR_RATIO = SwerveGearRatios.L2_DRIVE_KRAKEN;

    public static final double WHEEL_COF = 1.5;

    /**
     * Not every motor can output the max speed at all times, add a buffer to make closed loop more
     * accurate
     */
    public static final double MOTOR_CLOSED_LOOP_OUTPUT_SCALAR = 0.95;

    /** User defined acceleration time in seconds */
    public static final double ACCELERATION_TIME = 0.7;

    public static final double SLIP_CURRENT = 65.0;

    public static final double MAX_DRIVE_VELOCITY =
        ((Motors.KrakenX60Foc.FREE_SPEED / TAU) / DRIVE_GEAR_RATIO)
            * WHEEL_CIRCUMFERENCE
            * MOTOR_CLOSED_LOOP_OUTPUT_SCALAR;

    public static final double MAX_ANGULAR_VELOCITY = MAX_DRIVE_VELOCITY / DRIVEBASE_RADIUS;
    public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY / ACCELERATION_TIME;

    public static final double MAX_STEERING_VELOCITY =
        Motors.Falcon500Foc.FREE_SPEED / (STEER_GEAR_RATIO * MOTOR_CLOSED_LOOP_OUTPUT_SCALAR);

    /* Inverts */
    public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT =
        SensorDirectionValue.CounterClockwise_Positive;

    /* Neutral Modes */
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final class kDriveMotor {
      public static final double kP = 0.27;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double kS = 0.15;
      public static final double kV =
          12.0
              / (kSwerve.MAX_DRIVE_VELOCITY
                  / (kSwerve.WHEEL_CIRCUMFERENCE / kSwerve.DRIVE_GEAR_RATIO));
    }

    public static final class kSteerMotor {
      public static final double kP = 11.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double kS = 0.7;
    }

    public static final class kRotationController {
      public static final double kP = 9.0;
      public static final double kD = 0.2;

      public static final double CONSTRAINT_SCALAR = 1.0;
    }

    public static final boolean ORIENT_TELEOP_FOR_SIM_DEFAULT = false;

    public static final LerpTable TELEOP_TRANSLATION_AXIS_CURVE =
        new LerpTable(
            new LerpTableEntry(0.0, 0.0),
            new LerpTableEntry(0.15, 0.0), // deadzone
            new LerpTableEntry(0.7, 0.4),
            new LerpTableEntry(1.0, 1.0));

    public static final LerpTable TELEOP_ROTATION_AXIS_CURVE =
        new LerpTable(
            new LerpTableEntry(0.0, 0.0),
            new LerpTableEntry(0.15, 0.0), // deadzone
            new LerpTableEntry(0.5, 0.2),
            new LerpTableEntry(0.7, 0.4),
            new LerpTableEntry(1.0, 1.0));

    public static final double[] CRASH_ROTATION_OFFSETS =
        new double[] {-0.1015, 0.42529, -0.4182, -0.1086};

    public static final double[] BURN_ROTATION_OFFSETS =
        new double[] {0.0824, 0.10595, -0.21533, -0.398925};

    public static final Translation2d[] MODULE_CHASSIS_OFFSETS =
        new Translation2d[] {
          new Translation2d(TRACK_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(-TRACK_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(-TRACK_WIDTH / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(TRACK_WIDTH / 2.0, TRACK_WIDTH / 2.0)
        };

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(MODULE_CHASSIS_OFFSETS);
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
}
