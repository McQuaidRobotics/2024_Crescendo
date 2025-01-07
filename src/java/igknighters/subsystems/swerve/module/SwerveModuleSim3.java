package igknighters.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import igknighters.constants.ConstValues.kSwerve.kSteerMotor;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import sham.ShamSwerve;
import sham.shamController.ClosedLoop;
import sham.shamController.ShamMCX;
import sham.shamController.UnitSafeControl.AngularPIDFeedback;
import sham.shamController.UnitSafeControl.AngularVelocityPIDFeedback;
import sham.shamController.UnitSafeControl.FlywheelFeedforward;

public class SwerveModuleSim3 extends SwerveModule {

  private boolean gotDirectionsLastCycle = false;

  private final int moduleId;

  private final ShamMCX driveMotor;
  private final ShamMCX steerMotor;
  private final ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> driveLoop;
  private final ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> steerLoop;

  public SwerveModuleSim3(final int moduleId, SimSwerveOdometryThread odoThread, ShamSwerve sim) {
    super("SwerveModule[" + moduleId + "]");
    this.moduleId = moduleId;

    driveMotor = new ShamMCX("DriveMotor[" + moduleId + "]");
    steerMotor = new ShamMCX("SteerMotor[" + moduleId + "]");

    odoThread.addModulePositionSupplier(moduleId, this::getCurrentPosition);

    driveLoop =
        ClosedLoop.forVoltageAngularVelocity(
            new AngularVelocityPIDFeedback<VoltageUnit>(
                Volts.per(RotationsPerSecond).ofNative(kDriveMotor.kP),
                Volts.per(RotationsPerSecondPerSecond).ofNative(kDriveMotor.kD)),
            new FlywheelFeedforward<VoltageUnit>(
                Volts.of(kDriveMotor.kS),
                Volts.per(RotationsPerSecond).ofNative(kDriveMotor.kV),
                Volts.per(RotationsPerSecondPerSecond).ofNative(0.0)));

    steerLoop =
        ClosedLoop.forVoltageAngle(
            new AngularPIDFeedback<VoltageUnit>(
                    Volts.per(Rotations).ofNative(kSteerMotor.kP),
                    Volts.per(RotationsPerSecond).ofNative(kSteerMotor.kD))
                .withContinuousAngularInput(),
            new FlywheelFeedforward<VoltageUnit>(Volts.of(kSteerMotor.kS)),
            true);

    steerMotor.configSensorToMechanismRatio(kSwerve.STEER_GEAR_RATIO);

    sim.withSetModuleControllers(moduleId, driveMotor, steerMotor);

    BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "] initialized (sim)");
  }

  public void setDesiredState(AdvancedSwerveModuleState desiredState) {
    gotDirectionsLastCycle = true;
    setAngle(desiredState);
    setSpeed(desiredState);
  }

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(
        driveMotor.position().div(kSwerve.DRIVE_GEAR_RATIO).in(Rotations)
            * kSwerve.WHEEL_CIRCUMFERENCE,
        new Rotation2d(steerMotor.position()));
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
        driveMotor.velocity().div(kSwerve.DRIVE_GEAR_RATIO).in(RotationsPerSecond)
            * kSwerve.WHEEL_CIRCUMFERENCE,
        new Rotation2d(steerMotor.position()));
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
    steerMotor.controlVoltage(steerLoop, desiredState.angle.getMeasure());
  }

  private void setSpeed(SwerveModuleState desiredState) {
    super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
    driveMotor.controlVoltage(
        driveLoop,
        RotationsPerSecond.of(
            (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE)
                * kSwerve.DRIVE_GEAR_RATIO));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() || !gotDirectionsLastCycle) {
      targetDriveVeloMPS = 0.0;
    }
    log("gotDirectionsLastCycle", gotDirectionsLastCycle);
    gotDirectionsLastCycle = false;

    super.drivePositionMeters =
        driveMotor.position().div(kSwerve.DRIVE_GEAR_RATIO).in(Rotations)
            * kSwerve.WHEEL_CIRCUMFERENCE;
    super.driveVeloMPS =
        driveMotor.velocity().div(kSwerve.DRIVE_GEAR_RATIO).in(RotationsPerSecond)
            * kSwerve.WHEEL_CIRCUMFERENCE;

    super.steerAbsoluteRads = steerMotor.position().in(Radians);
    super.steerVeloRadPS = steerMotor.velocity().in(RadiansPerSecond);

    super.driveVolts = driveMotor.voltage().in(Volts);
    super.steerVolts = steerMotor.voltage().in(Volts);
    super.driveAmps = driveMotor.current().in(Amps);
    super.steerAmps = steerMotor.current().in(Amps);
  }

  @Override
  public void setVoltageOut(double volts, Rotation2d angle) {
    super.targetSteerAbsoluteRads = angle.getRadians();
    driveMotor.controlVoltage(Volts.of(volts));
    steerMotor.controlVoltage(steerLoop, angle.getMeasure());
  }
}
