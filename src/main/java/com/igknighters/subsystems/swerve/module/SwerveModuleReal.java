package com.igknighters.subsystems.swerve.module;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kSwerve.kAngleMotor;
import com.igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.CANRetrier;

public class SwerveModuleReal implements SwerveModule {
    private final TalonFX driveMotor;
    private final StatusSignal<Double> drivePositionSignal, driveVelocitySignal;
    private final StatusSignal<Double> driveVoltSignal, driveAmpSignal;

    private final TalonFX angleMotor;
    private final StatusSignal<Double> anglePositionSignal, angleVelocitySignal;
    private final StatusSignal<Double> angleVoltSignal, angleAmpSignal;

    private final CANcoder angleEncoder;
    private final StatusSignal<Double> angleAbsoluteSignal, angleAbsoluteVeloSignal;

    public final int moduleNumber;
    private final double rotationOffset;
    @SuppressWarnings("unused")
    private final Translation2d moduleChassisPose;
    private Rotation2d lastAngle = new Rotation2d();
    private final SwerveModuleInputs inputs;
    private final boolean isPro;

    public SwerveModuleReal(final SwerveModuleConstants moduleConstants, boolean isPro) {
        this.isPro = isPro;
        this.moduleNumber = moduleConstants.getModuleId().num;
        this.rotationOffset = moduleConstants.getRotationOffset();
        this.moduleChassisPose = moduleConstants.getModuleChassisPose();

        driveMotor = new TalonFX(moduleConstants.getDriveMotorID(), kSwerve.CANBUS);
        angleMotor = new TalonFX(moduleConstants.getAngleMotorID(), kSwerve.CANBUS);
        angleEncoder = new CANcoder(moduleConstants.getCancoderID(), kSwerve.CANBUS);

        CANRetrier.retryStatusCode(() -> driveMotor.getConfigurator().apply(driveMotorConfig()), 5);
        CANRetrier.retryStatusCode(() -> angleMotor.getConfigurator().apply(angleMotorConfig()), 5);
        CANRetrier.retryStatusCode(() -> angleEncoder.getConfigurator().apply(cancoderConfig()), 5);

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveVoltSignal = driveMotor.getMotorVoltage();
        driveAmpSignal = driveMotor.getTorqueCurrent();

        drivePositionSignal.setUpdateFrequency(100);
        driveVelocitySignal.setUpdateFrequency(100);
        driveVoltSignal.setUpdateFrequency(100);
        driveAmpSignal.setUpdateFrequency(100);

        driveMotor.optimizeBusUtilization();

        anglePositionSignal = angleMotor.getPosition();
        angleVelocitySignal = angleMotor.getVelocity();
        angleVoltSignal = angleMotor.getMotorVoltage();
        angleAmpSignal = angleMotor.getTorqueCurrent();

        anglePositionSignal.setUpdateFrequency(100);
        angleVelocitySignal.setUpdateFrequency(100);
        angleVoltSignal.setUpdateFrequency(100);
        angleAmpSignal.setUpdateFrequency(100);

        angleMotor.optimizeBusUtilization();

        angleAbsoluteSignal = angleEncoder.getAbsolutePosition();
        angleAbsoluteVeloSignal = angleEncoder.getVelocity();

        angleAbsoluteSignal.setUpdateFrequency(100);
        angleAbsoluteVeloSignal.setUpdateFrequency(100);

        angleEncoder.optimizeBusUtilization();

        driveMotor.setPosition(0.0);

        inputs = new SwerveModuleInputs();

        BootupLogger.bootupLog(
                "    SwerveModule[" + this.moduleNumber + "] initialized ("
                        + (isPro ? " realPro" : "real")
                        + ")");
    }

    private TalonFXConfiguration driveMotorConfig() {
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

    private TalonFXConfiguration angleMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = kSwerve.ANGLE_MOTOR_INVERT;
        cfg.MotorOutput.NeutralMode = kSwerve.ANGLE_NEUTRAL_MODE;

        cfg.Slot0.kP = kAngleMotor.kP;
        cfg.Slot0.kI = kAngleMotor.kI;
        cfg.Slot0.kD = kAngleMotor.kD;

        cfg.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        cfg.Feedback.RotorToSensorRatio = kSwerve.ANGLE_GEAR_RATIO;
        if (isPro) {
            cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        } else {
            cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        }
        cfg.ClosedLoopGeneral.ContinuousWrap = true;

        return cfg;
    }

    private CANcoderConfiguration cancoderConfig() {
        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = rotationOffset;

        return canCoderConfig;
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;
        inputs.targetAngleAbsoluteRads = angle.getRadians();

        angleMotor.setControl(
                new PositionDutyCycle(angle.getRotations())
                        .withUpdateFreqHz(250));
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        inputs.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_DRIVE_VELOCITY;
            var controlRequest = new DutyCycleOut(percentOutput).withEnableFOC(isPro);
            driveMotor.setControl(controlRequest);
        } else {
            double rps = (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE) * kSwerve.DRIVE_GEAR_RATIO;
            var veloRequest = new VelocityVoltage(rps).withEnableFOC(isPro);
            Logger.recordOutput("Swerve/SwerveModule[" + this.moduleNumber + "]/DriveRPS", rps);
            driveMotor.setControl(veloRequest);
        }
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                inputs.driveVeloMPS,
                getAngle());
    }

    @Override
    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                inputs.drivePositionMeters,
                getAngle());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.angleAbsoluteRads);
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                drivePositionSignal, driveVelocitySignal,
                /* driveVoltSignal, driveAmpSignal, */
                anglePositionSignal, angleVelocitySignal,
                /* angleVoltSignal, angleAmpSignal , */
                angleAbsoluteSignal /* , angleAbsoluteVeloSignal */);

        inputs.angleAbsoluteRads = Units.rotationsToRadians(angleAbsoluteSignal.getValue());
        inputs.angleVeloRadPS = Units.rotationsToRadians(angleAbsoluteVeloSignal.getValue());
        inputs.angleVolts = angleVoltSignal.getValue();
        inputs.angleAmps = angleAmpSignal.getValue();

        inputs.drivePositionMeters = driveRotationsToMeters(drivePositionSignal.getValue());
        inputs.driveVeloMPS = driveRotationsToMeters(driveVelocitySignal.getValue());
        inputs.driveVolts = driveVoltSignal.getValue();
        inputs.driveAmps = driveAmpSignal.getValue();

        Logger.processInputs("Swerve/SwerveModule[" + this.moduleNumber + "]", inputs);
    }

    @Override
    public void setVoltageOut(double volts) {
    }
}
