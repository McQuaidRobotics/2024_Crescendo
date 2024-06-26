package com.igknighters.subsystems.swerve.module;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kSwerve.kAngleMotor;
import com.igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;

public class SwerveModuleReal extends SwerveModule {
    private final TalonFX driveMotor;
    private final StatusSignal<Double> drivePositionSignal, driveVelocitySignal;
    private final StatusSignal<Double> driveVoltSignal, driveAmpSignal;
    private final ControlRequest driveMotorClosedReq;
    private final ControlRequest driveMotorOpenReq;

    private final TalonFX angleMotor;
    private final StatusSignal<Double> anglePositionSignal, angleVelocitySignal;
    private final StatusSignal<Double> angleVoltSignal, angleAmpSignal;
    private final PositionDutyCycle angleMotorReq = new PositionDutyCycle(0)
            .withUpdateFreqHz(0);

    private final CANcoder angleEncoder;
    private final StatusSignal<Double> angleAbsoluteSignal, angleAbsoluteVeloSignal;

    public final int moduleNumber;
    private final double rotationOffset;
    private final boolean isPro;

    private Rotation2d lastAngle = new Rotation2d();

    public SwerveModuleReal(final SwerveModuleConstants moduleConstants, boolean isPro) {
        this.isPro = isPro;
        this.moduleNumber = moduleConstants.getModuleId().num;
        this.rotationOffset = moduleConstants.getRotationOffset();

        driveMotor = new TalonFX(moduleConstants.getDriveMotorID(), kSwerve.CANBUS);
        angleMotor = new TalonFX(moduleConstants.getAngleMotorID(), kSwerve.CANBUS);
        angleEncoder = new CANcoder(moduleConstants.getCancoderID(), kSwerve.CANBUS);

        CANRetrier.retryStatusCode(() -> driveMotor.getConfigurator().apply(driveMotorConfig(), 1.0), 5);
        CANRetrier.retryStatusCode(() -> angleMotor.getConfigurator().apply(angleMotorConfig(), 1.0), 5);
        CANRetrier.retryStatusCode(() -> angleEncoder.getConfigurator().apply(cancoderConfig(), 1.0), 5);

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveVoltSignal = driveMotor.getMotorVoltage();
        driveAmpSignal = driveMotor.getTorqueCurrent();

        anglePositionSignal = angleMotor.getPosition();
        angleVelocitySignal = angleMotor.getVelocity();
        angleVoltSignal = angleMotor.getMotorVoltage();
        angleAmpSignal = angleMotor.getTorqueCurrent();

        angleAbsoluteSignal = angleEncoder.getAbsolutePosition();
        angleAbsoluteVeloSignal = angleEncoder.getVelocity();

        CANSignalManager.registerSignals(
            kSwerve.CANBUS,
            drivePositionSignal, driveVelocitySignal,
            driveVoltSignal, driveAmpSignal,
            anglePositionSignal, angleVelocitySignal,
            angleVoltSignal, angleAmpSignal,
            angleAbsoluteSignal, angleAbsoluteVeloSignal
        );

        driveMotor.optimizeBusUtilization(1.0);
        angleMotor.optimizeBusUtilization(1.0);
        angleEncoder.optimizeBusUtilization(1.0);

        CANRetrier.retryStatusCode(() -> driveMotor.setPosition(0.0, 0.1), 3);

        log("isPro", isPro);

        driveMotorOpenReq = new VoltageOut(0).withEnableFOC(isPro).withUpdateFreqHz(0);
        // if (isPro) {
        //     driveMotorClosedReq = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
        // } else {
        //     driveMotorClosedReq = new VelocityVoltage(0).withUpdateFreqHz(0);
        // }
        driveMotorClosedReq = new VelocityVoltage(0).withUpdateFreqHz(0);

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
        super.targetAngleAbsoluteRads = angle.getRadians();

        angleMotor.setControl(angleMotorReq.withPosition(angle.getRotations()));
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_DRIVE_VELOCITY;
            driveMotor.setControl(((VoltageOut) driveMotorOpenReq).withOutput(percentOutput * RobotController.getBatteryVoltage()));
        } else {
            double rps = (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE) * kSwerve.DRIVE_GEAR_RATIO;
            log("DriveRPS", rps);
            if (isPro) {
                driveMotor.setControl(((VelocityTorqueCurrentFOC) driveMotorClosedReq).withVelocity(rps));
            } else {
                driveMotor.setControl(((VelocityVoltage) driveMotorClosedReq).withVelocity(rps));
            }
        }
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
        return Rotation2d.fromRadians(super.angleAbsoluteRads);
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public void periodic() {
        super.angleAbsoluteRads = Units.rotationsToRadians(angleAbsoluteSignal.getValue());
        super.angleVeloRadPS = Units.rotationsToRadians(angleAbsoluteVeloSignal.getValue());
        super.angleVolts = angleVoltSignal.getValue();
        super.angleAmps = angleAmpSignal.getValue();

        super.drivePositionMeters = driveRotationsToMeters(drivePositionSignal.getValue());
        super.driveVeloMPS = driveRotationsToMeters(driveVelocitySignal.getValue());
        super.driveVolts = driveVoltSignal.getValue();
        super.driveAmps = driveAmpSignal.getValue();
    }

    @Override
    public void setVoltageOut(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setVoltageOut(double volts, Rotation2d angle) {
        setAngle(new SwerveModuleState(0.0, angle));
        driveMotor.setVoltage(volts);
    }
}
