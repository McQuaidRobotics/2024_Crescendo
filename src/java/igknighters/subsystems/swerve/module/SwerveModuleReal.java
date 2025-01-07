package igknighters.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import igknighters.constants.ConstValues.kSwerve.kSteerMotor;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.util.can.CANRetrier;
import igknighters.util.can.CANSignalManager;
import igknighters.util.logging.BootupLogger;
import monologue.Annotations.IgnoreLogged;

public class SwerveModuleReal extends SwerveModule {
  private final TalonFX driveMotor;
  private final BaseStatusSignal driveVoltSignal, driveAmpSignal;
  private final VelocityVoltage driveMotorClosedReq;

  private final TalonFX steerMotor;
  private final BaseStatusSignal steerVoltSignal, steerAmpSignal;
  private final PositionDutyCycle steerMotorReq = new PositionDutyCycle(0).withUpdateFreqHz(0);

  private final CANcoder steerEncoder;
  private final BaseStatusSignal steerAbsoluteSignal, steerAbsoluteVeloSignal;

  public final int moduleId;

  @IgnoreLogged private final RealSwerveOdometryThread odoThread;

  private Rotation2d lastAngle = new Rotation2d();

  public SwerveModuleReal(final int moduleId, final RealSwerveOdometryThread odoThread) {
    super("SwerveModule[" + moduleId + "]");
    this.odoThread = odoThread;

    this.moduleId = moduleId;
    double rotationOffset = super.getOffset(moduleId);

    driveMotor = new TalonFX((moduleId * 2) + 1, kSwerve.CANBUS);
    steerMotor = new TalonFX((moduleId * 2) + 2, kSwerve.CANBUS);
    steerEncoder = new CANcoder(21 + moduleId, kSwerve.CANBUS);

    CANRetrier.retryStatusCode(
        () -> driveMotor.getConfigurator().apply(driveMotorConfig(), 1.0), 5);
    CANRetrier.retryStatusCode(
        () -> steerMotor.getConfigurator().apply(steerMotorConfig(steerEncoder.getDeviceID()), 1.0),
        5);
    CANRetrier.retryStatusCode(
        () -> steerEncoder.getConfigurator().apply(cancoderConfig(rotationOffset), 1.0), 5);

    driveVoltSignal = driveMotor.getMotorVoltage();
    driveAmpSignal = driveMotor.getTorqueCurrent();

    steerVoltSignal = steerMotor.getMotorVoltage();
    steerAmpSignal = steerMotor.getTorqueCurrent();

    steerAbsoluteSignal = steerEncoder.getAbsolutePosition();
    steerAbsoluteVeloSignal = steerEncoder.getVelocity();

    CANSignalManager.registerSignals(
        kSwerve.CANBUS,
        driveVoltSignal,
        driveAmpSignal,
        steerVoltSignal,
        steerAmpSignal,
        steerAbsoluteSignal,
        steerAbsoluteVeloSignal);

    odoThread.addModuleStatusSignals(
        moduleId,
        driveMotor.getPosition(),
        driveMotor.getVelocity(),
        steerMotor.getPosition(),
        steerMotor.getVelocity());

    driveMotor.optimizeBusUtilization(0.0, 1.0);
    steerMotor.optimizeBusUtilization(0.0, 1.0);
    steerEncoder.optimizeBusUtilization(0.0, 1.0);

    CANRetrier.retryStatusCode(() -> driveMotor.setPosition(0.0, 0.1), 3);

    driveMotorClosedReq = new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);

    BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "]");
  }

  protected TalonFXConfiguration driveMotorConfig() {
    var cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted = kSwerve.DRIVE_MOTOR_INVERT;
    cfg.MotorOutput.NeutralMode = kSwerve.DRIVE_NEUTRAL_MODE;

    cfg.Slot0.kP = kDriveMotor.kP;
    cfg.Slot0.kI = kDriveMotor.kI;
    cfg.Slot0.kD = kDriveMotor.kD;
    cfg.Slot0.kV =
        12.0
            / (kSwerve.MAX_DRIVE_VELOCITY
                / (kSwerve.WHEEL_CIRCUMFERENCE / kSwerve.DRIVE_GEAR_RATIO));
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

  public int getModuleId() {
    return this.moduleId;
  }

  @Override
  public void setDesiredState(AdvancedSwerveModuleState desiredState) {
    desiredState.optimize(getAngle());
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
            ? lastAngle
            : desiredState.angle;
    super.targetSteerAbsoluteRads = angle.getRadians();

    steerMotor.setControl(steerMotorReq.withPosition(angle.getRotations()));
    lastAngle = angle;
  }

  private void setSpeed(AdvancedSwerveModuleState desiredState) {
    super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
    double rps =
        (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE)
            * kSwerve.DRIVE_GEAR_RATIO;
    log("DriveRPS", rps);
    driveMotor.setControl(
        driveMotorClosedReq.withVelocity(rps).withAcceleration(desiredState.driveAccelerationFF));
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(super.driveVeloMPS, getAngle());
  }

  @Override
  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(super.drivePositionMeters, getAngle());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRadians(super.steerAbsoluteRads);
  }

  private double driveRotationsToMeters(double rotations) {
    return (rotations / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE;
  }

  @Override
  public void periodic() {
    super.steerAbsoluteRads = Units.rotationsToRadians(steerAbsoluteSignal.getValueAsDouble());
    super.steerVeloRadPS = Units.rotationsToRadians(steerAbsoluteVeloSignal.getValueAsDouble());
    super.steerVolts = steerVoltSignal.getValueAsDouble();
    super.steerAmps = steerAmpSignal.getValueAsDouble();

    super.drivePositionMeters = driveRotationsToMeters(odoThread.getModulePosition(moduleId));
    super.driveVeloMPS = driveRotationsToMeters(odoThread.getModuleVelocity(moduleId));
    super.driveVolts = driveVoltSignal.getValueAsDouble();
    super.driveAmps = driveAmpSignal.getValueAsDouble();
  }

  @Override
  public void setVoltageOut(double volts, Rotation2d angle) {
    setAngle(new SwerveModuleState(0.0, angle));
    driveMotor.setVoltage(volts);
  }
}
