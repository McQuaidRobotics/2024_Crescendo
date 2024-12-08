package igknighters.subsystems.swerve.module;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import sham.ShamDriveTrainSwerve;
import sham.utils.GearRatio;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import monologue.Annotations.IgnoreLogged;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.util.can.CANRetrier;
import igknighters.util.can.CANSignalManager;
import igknighters.util.logging.BootupLogger;
import igknighters.util.sim.FusedTalonFxSimController;
import igknighters.util.sim.TalonFxSimController;

public class SwerveModuleOmni extends SwerveModule {
    private final TalonFX driveMotor;
    private final BaseStatusSignal driveVoltSignal, driveAmpSignal;
    private final VelocityVoltage driveMotorClosedReq = new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);

    private final TalonFX steerMotor;
    private final BaseStatusSignal angleVoltSignal, angleAmpSignal;
    private final PositionDutyCycle angleMotorReq = new PositionDutyCycle(0)
            .withUpdateFreqHz(0);

    private final CANcoder steerEncoder;
    private final BaseStatusSignal angleAbsoluteSignal, angleAbsoluteVeloSignal;

    public final int moduleId;

    @IgnoreLogged
    private final RealSwerveOdometryThread odoThread;

    public SwerveModuleOmni(final int moduleId, final RealSwerveOdometryThread odoThread, final SimCtx simCtx) {
        super("SwerveModule[" + moduleId + "]");
        this.odoThread = odoThread;

        this.moduleId = moduleId;
        double rotationOffset = super.getOffset(moduleId);

        driveMotor = new TalonFX((moduleId * 2) + 1, kSwerve.CANBUS);
        steerMotor = new TalonFX((moduleId * 2) + 2, kSwerve.CANBUS);
        steerEncoder = new CANcoder(21 + moduleId, kSwerve.CANBUS);

        CANRetrier.retryStatusCode(() -> driveMotor.getConfigurator().apply(driveMotorConfig(), 1.0), 5);
        CANRetrier.retryStatusCode(() -> steerMotor.getConfigurator().apply(steerMotorConfig(steerEncoder.getDeviceID()), 1.0), 5);
        CANRetrier.retryStatusCode(() -> steerEncoder.getConfigurator().apply(cancoderConfig(rotationOffset), 1.0), 5);

        if (simCtx.isActive()) {
            ((ShamDriveTrainSwerve) simCtx.robot().getDriveTrain())
                .withSetModuleControllers(
                    moduleId,
                    new TalonFxSimController(driveMotor.getSimState()).withBrakeEnabled(true),
                    new FusedTalonFxSimController(steerMotor.getSimState(), steerEncoder.getSimState(), GearRatio.reduction(kSwerve.STEER_GEAR_RATIO))
                );
        }

        driveVoltSignal = driveMotor.getMotorVoltage();
        driveAmpSignal = driveMotor.getTorqueCurrent();

        angleVoltSignal = steerMotor.getMotorVoltage();
        angleAmpSignal = steerMotor.getTorqueCurrent();

        angleAbsoluteSignal = steerEncoder.getAbsolutePosition();
        angleAbsoluteVeloSignal = steerEncoder.getVelocity();

        CANSignalManager.registerSignals(
            kSwerve.CANBUS,
            driveVoltSignal, driveAmpSignal,
            angleVoltSignal, angleAmpSignal,
            angleAbsoluteSignal, angleAbsoluteVeloSignal
        );

        odoThread.addModuleStatusSignals(
            moduleId,
            driveMotor.getPosition(),
            driveMotor.getVelocity(),
            steerMotor.getPosition(),
            steerMotor.getVelocity()
        );

        driveMotor.optimizeBusUtilization(0.0, 1.0);
        steerMotor.optimizeBusUtilization(0.0, 1.0);
        steerEncoder.optimizeBusUtilization(0.0, 1.0);

        CANRetrier.retryStatusCode(() -> driveMotor.setPosition(0.0, 0.1), 3);

        BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "] initialized (omni)");
    }

    @Override
    public int getModuleId() {
        return this.moduleId;
    }

    @Override
    public void setDesiredState(AdvancedSwerveModuleState desiredState) {
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    private void setAngle(SwerveModuleState desiredState) {
        super.targetSteerAbsoluteRads = desiredState.angle.getRadians();

        if (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01)) {
            return;
        }

        steerMotor.setControl(angleMotorReq.withPosition(desiredState.angle.getRotations()));
    }

    private void setSpeed(AdvancedSwerveModuleState desiredState) {
        super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
        double rps = (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE) * kSwerve.DRIVE_GEAR_RATIO;
        log("DriveRPS", rps);
        driveMotor.setControl(
            driveMotorClosedReq.withVelocity(rps));
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                super.driveVeloMPS,
                getAngle());
    }

    @Override
    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                super.drivePositionMeters,
                getAngle());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(super.steerAbsoluteRads);
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public void periodic() {
        super.steerAbsoluteRads = Units.rotationsToRadians(angleAbsoluteSignal.getValueAsDouble());
        super.steerVeloRadPS = Units.rotationsToRadians(angleAbsoluteVeloSignal.getValueAsDouble());
        super.steerVolts = angleVoltSignal.getValueAsDouble();
        super.steerAmps = angleAmpSignal.getValueAsDouble();

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
