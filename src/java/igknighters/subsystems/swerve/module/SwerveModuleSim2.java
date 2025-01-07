package igknighters.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import igknighters.constants.ConstValues.kSwerve.kSteerMotor;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import sham.ShamMechanism.MechanismState;
import sham.ShamMotorController;
import sham.ShamMotorController.ControllerOutput.VoltageOutput;
import sham.ShamSwerve;

public class SwerveModuleSim2 extends SwerveModule {

  private boolean gotDirectionsLastCycle = false;

  private final PIDController driveFeedback;
  private final PIDController steerFeedback;
  private final SimpleMotorFeedforward driveFeedforward;

  public final int moduleId;

  private MechanismState driveState = MechanismState.zero();
  private MechanismState steerState = MechanismState.zero();

  private Voltage driveAppliedVoltage = Volts.zero();
  private Voltage steerAppliedVoltage = Volts.zero();

  public SwerveModuleSim2(final int moduleId, SimSwerveOdometryThread odoThread, ShamSwerve sim) {
    super("SwerveModule[" + moduleId + "]");
    this.moduleId = moduleId;

    odoThread.addModulePositionSupplier(moduleId, this::getCurrentPosition);

    driveFeedback =
        new PIDController(
            kDriveMotor.kP, kDriveMotor.kI, kDriveMotor.kD, sim.timing().dt().in(Seconds));
    driveFeedforward =
        new SimpleMotorFeedforward(
            kDriveMotor.kS, kDriveMotor.kV, 0.0, sim.timing().dt().in(Seconds));

    steerFeedback =
        new PIDController(
            kSteerMotor.kP, kSteerMotor.kI, kSteerMotor.kD, sim.timing().dt().in(Seconds));
    steerFeedback.enableContinuousInput(-0.5, 0.5);

    ShamMotorController driveController =
        new ShamMotorController() {
          @Override
          public boolean brakeEnabled() {
            return true;
          }

          @Override
          public VoltageOutput run(Time dt, Voltage supply, MechanismState state) {
            driveState = state;

            double velocityRadPerSec =
                (targetDriveVeloMPS / kSwerve.WHEEL_RADIUS) * kSwerve.DRIVE_GEAR_RATIO;
            var feedback =
                driveFeedback.calculate(
                    driveState.velocity().in(RotationsPerSecond),
                    Units.radiansToRotations(velocityRadPerSec));
            var feedforward =
                driveFeedforward.calculateWithVelocities(
                    driveState.velocity().in(RotationsPerSecond),
                    Units.radiansToRotations(velocityRadPerSec));
            var output =
                MathUtil.clamp(
                    feedback + feedforward,
                    -RobotController.getBatteryVoltage(),
                    RobotController.getBatteryVoltage());

            return VoltageOutput.of(output);
          }
        };

    ShamMotorController steerController =
        new ShamMotorController() {
          @Override
          public boolean brakeEnabled() {
            return false;
          }

          @Override
          public VoltageOutput run(Time dt, Voltage supply, MechanismState state) {
            steerState = state.div(kSwerve.STEER_GEAR_RATIO);

            var feedback =
                MathUtil.clamp(
                    steerFeedback.calculate(
                        steerState.position().in(Rotations),
                        Units.radiansToRotations(targetSteerAbsoluteRads)),
                    -1.0,
                    1.0);

            return VoltageOutput.of(feedback * RobotController.getBatteryVoltage());
          }
        };

    sim.withSetModuleControllers(moduleId, driveController, steerController);

    BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "] initialized (sim)");
  }

  public void setDesiredState(AdvancedSwerveModuleState desiredState) {
    gotDirectionsLastCycle = true;
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(
        super.drivePositionMeters, new Rotation2d(steerState.position()));
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(super.driveVeloMPS, new Rotation2d(steerState.position()));
  }

  @Override
  public int getModuleId() {
    return this.moduleId;
  }

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
            ? new Rotation2d(super.steerAbsoluteRads)
            : desiredState.angle;
    super.targetSteerAbsoluteRads = angle.getRadians();
  }

  private void setSpeed(SwerveModuleState desiredState) {
    super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !gotDirectionsLastCycle) {
      targetDriveVeloMPS = 0.0;
    }
    log("gotDirectionsLastCycle", gotDirectionsLastCycle);
    gotDirectionsLastCycle = false;

    // super.drivePositionMeters += driveRadiansToMeters(
    //         driveSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME);

    // double angleDiffRad = angleSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME;
    // super.angleAbsoluteRads += angleDiffRad;

    // while (super.angleAbsoluteRads < 0) {
    //     super.angleAbsoluteRads += 2 * Math.PI;
    // }
    // while (super.angleAbsoluteRads > 2 * Math.PI) {
    //     super.angleAbsoluteRads -= 2 * Math.PI;
    // }

    // super.angleVeloRadPS = angleSim.getAngularVelocityRadPerSec();
    // super.angleAmps = angleSim.getCurrentDrawAmps();

    // super.driveVeloMPS = driveRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0);
    // super.driveAmps = driveSim.getCurrentDrawAmps();

    super.drivePositionMeters =
        driveState.position().div(kSwerve.DRIVE_GEAR_RATIO).in(Rotations)
            * kSwerve.WHEEL_CIRCUMFERENCE;
    super.driveVeloMPS =
        driveState.velocity().div(kSwerve.DRIVE_GEAR_RATIO).in(RotationsPerSecond)
            * kSwerve.WHEEL_CIRCUMFERENCE;

    super.steerAbsoluteRads = steerState.position().in(Radians);
    super.steerVeloRadPS = steerState.velocity().in(RadiansPerSecond);

    super.driveVolts = driveAppliedVoltage.in(Volts);
    super.steerVolts = steerAppliedVoltage.in(Volts);
  }

  @Override
  public void setVoltageOut(double volts, Rotation2d angle) {
    driveAppliedVoltage = Volts.of(volts);
    super.steerAbsoluteRads = angle.getRadians();
    super.targetSteerAbsoluteRads = angle.getRadians();
  }
}
