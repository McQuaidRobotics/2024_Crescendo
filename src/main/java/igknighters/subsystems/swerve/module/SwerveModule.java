package igknighters.subsystems.swerve.module;

import igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import igknighters.constants.ConstValues.kSwerve.kSteerMotor;
import igknighters.constants.RobotConfig;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.subsystems.Component;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import monologue.Annotations.Log;

public abstract class SwerveModule extends Component {
    public static final class AdvancedSwerveModuleState extends SwerveModuleState {
        public double steerVelocityFF;
        public double driveAccelerationFF;

        public AdvancedSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double steerVelocityFF,
                double driveAccelerationFF) {
            super(speedMetersPerSecond, angle);
            this.steerVelocityFF = steerVelocityFF;
            this.driveAccelerationFF = driveAccelerationFF;
        }

        //todo: implement custom struct
        public static final Struct<SwerveModuleState> struct = SwerveModuleState.struct;

        public static AdvancedSwerveModuleState fromBase(SwerveModuleState base) {
            return new AdvancedSwerveModuleState(base.speedMetersPerSecond, base.angle, 0.0, 0.0);
        }
    }

    @Log
    public double driveVeloMPS = 0.0;
    @Log
    public double targetDriveVeloMPS = 0.0;
    @Log
    public double drivePositionMeters = 0.0;
    @Log
    public double driveVolts = 0.0;
    @Log
    public double driveAmps = 0.0;
    @Log
    public double angleVeloRadPS = 0.0;
    @Log
    public double angleAbsoluteRads = 0.0;
    @Log
    public double targetAngleAbsoluteRads = 0.0;
    @Log
    public double angleVolts = 0.0;
    @Log
    public double angleAmps = 0.0;

    public final String name;

    protected SwerveModule(String name) {
        this.name = name;
    }

    /**
     * @param desiredState The state that the module should assume, angle and
     *                     velocity.
     * @param isOpenLoop   Whether the module speed assumed should be reached via
     *                     open or closed loop control.
     */
    public abstract void setDesiredState(AdvancedSwerveModuleState desiredState);

    /**
     * @return The velocity/angle of the module.
     */
    public abstract SwerveModuleState getCurrentState();

    public abstract SwerveModulePosition getCurrentPosition();

    public abstract int getModuleId();

    public abstract void setVoltageOut(double volts, Rotation2d angle);

    protected TalonFXConfiguration driveMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = kSwerve.DRIVE_MOTOR_INVERT;
        cfg.MotorOutput.NeutralMode = kSwerve.DRIVE_NEUTRAL_MODE;

        cfg.Slot0.kP = kDriveMotor.kP;
        cfg.Slot0.kI = kDriveMotor.kI;
        cfg.Slot0.kD = kDriveMotor.kD;
        cfg.Slot0.kV = 12.0
                / (kSwerve.MAX_DRIVE_VELOCITY / (kSwerve.WHEEL_CIRCUMFERENCE / kSwerve.DRIVE_GEAR_RATIO));
        cfg.Slot0.kS = kDriveMotor.kS;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = kSwerve.SLIP_CURRENT;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = kSwerve.SLIP_CURRENT;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -kSwerve.SLIP_CURRENT;

        return cfg;
    }

    protected TalonFXConfiguration steerMotorConfig(int encoderId) {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = kSwerve.ANGLE_MOTOR_INVERT;
        cfg.MotorOutput.NeutralMode = kSwerve.ANGLE_NEUTRAL_MODE;

        cfg.Slot0.kP = kSteerMotor.kP;
        cfg.Slot0.kI = kSteerMotor.kI;
        cfg.Slot0.kD = kSteerMotor.kD;

        cfg.Feedback.FeedbackRemoteSensorID = encoderId;
        cfg.Feedback.RotorToSensorRatio = kSwerve.STEER_GEAR_RATIO;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.ClosedLoopGeneral.ContinuousWrap = true;

        return cfg;
    }

    protected CANcoderConfiguration cancoderConfig(double rotationOffset) {
        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = rotationOffset;

        return canCoderConfig;
    }

    protected double getOffset(int moduleId) {
        double[] offsetStore = switch (RobotConfig.getRobotID()) {
            case CRASH -> kSwerve.CRASH_ROTATION_OFFSETS;
            default -> kSwerve.BURN_ROTATION_OFFSETS;
        };

        return offsetStore[moduleId];
    }
}