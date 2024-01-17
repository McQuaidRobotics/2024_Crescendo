package com.igknighters.subsystems.swerve.module;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
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
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kSwerve.AngleMotorConstants;
import com.igknighters.constants.ConstValues.kSwerve.DriveMotorConstants;
import com.igknighters.util.SwerveModuleConstants;

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
    private final Rotation2d rotationOffset;
    @SuppressWarnings("unused")
    private final Translation2d moduleChassisPose;
    private Rotation2d lastAngle = new Rotation2d();
    private final SwerveModuleInputs inputs;

    public SwerveModuleReal(final SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleConstants.moduleId.num;
        this.rotationOffset = moduleConstants.getRotationOffset(moduleConstants.moduleId);
        this.moduleChassisPose = moduleConstants.moduleChassisPose;

        driveMotor = new TalonFX(moduleConstants.driveMotorID, kSwerve.CANBUS);
        angleMotor = new TalonFX(moduleConstants.angleMotorID, kSwerve.CANBUS);
        angleEncoder = new CANcoder(moduleConstants.cancoderID, kSwerve.CANBUS);

        configureDriveMotor();
        configureAngleMotor();
        configureCANcoder();

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

        inputs = new SwerveModuleInputs();
    }

    private void configureDriveMotor() {
        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = kSwerve.DRIVE_MOTOR_INVERT;
        driveConfig.MotorOutput.NeutralMode = kSwerve.DRIVE_NEUTRAL_MODE;
        driveConfig.Slot0.kP = DriveMotorConstants.kP;
        driveConfig.Slot0.kI = DriveMotorConstants.kI;
        driveConfig.Slot0.kD = DriveMotorConstants.kD;
        driveConfig.Slot0.kV = 12.0 / (kSwerve.MAX_DRIVE_VELOCITY / (kSwerve.WHEEL_CIRCUMFERENCE * kSwerve.DRIVE_GEAR_RATIO));
        driveConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveMotor.getConfigurator().apply(driveConfig);
    }

    private void configureAngleMotor() {
        var angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.Inverted = kSwerve.ANGLE_MOTOR_INVERT;
        angleConfig.MotorOutput.NeutralMode = kSwerve.ANGLE_NEUTRAL_MODE;
        angleConfig.Slot0.kP = AngleMotorConstants.kP;
        angleConfig.Slot0.kI = AngleMotorConstants.kI;
        angleConfig.Slot0.kD = AngleMotorConstants.kD;
        angleConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfig.Feedback.RotorToSensorRatio = 1.0 / kSwerve.ANGLE_GEAR_RATIO;
        angleConfig.Feedback.SensorToMechanismRatio = 1.0;
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

        angleMotor.getConfigurator().apply(angleConfig);
    }

    private void configureCANcoder() {
        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfig.MagnetSensor.SensorDirection = kSwerve.CANCODER_INVERT;
        canCoderConfig.MagnetSensor.MagnetOffset = -rotationOffset.getRotations();

        angleEncoder.getConfigurator().apply(canCoderConfig);
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
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01)) ? lastAngle
                : desiredState.angle;
        inputs.targetAngleAbsolute = angle.getRadians();

        angleMotor.setControl(new PositionDutyCycle(angle.getRotations()));
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        inputs.targetDriveVelo = desiredState.speedMetersPerSecond;
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_DRIVE_VELOCITY;
            var controlRequest = new DutyCycleOut(percentOutput).withEnableFOC(true);
            driveMotor.setControl(controlRequest);
        } else {
            double rps = desiredState.speedMetersPerSecond / (kSwerve.WHEEL_CIRCUMFERENCE * kSwerve.DRIVE_GEAR_RATIO);
            var veloRequest = new VelocityVoltage(rps).withEnableFOC(true);
            driveMotor.setControl(veloRequest);
        }
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                inputs.driveVelo,
                getAngle());
    }

    @Override
    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                inputs.drivePosition,
                getAngle());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.angleAbsolute);
    }

    private double driveRotationsToMeters(double rotations) {
        return rotations * (kSwerve.WHEEL_CIRCUMFERENCE * kSwerve.DRIVE_GEAR_RATIO);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            drivePositionSignal, driveVelocitySignal,
            driveVoltSignal, driveAmpSignal,
            anglePositionSignal, angleVelocitySignal,
            angleVoltSignal, angleAmpSignal,
            angleAbsoluteSignal, angleAbsoluteVeloSignal
        );

        inputs.angleAbsolute = angleAbsoluteSignal.getValue() * (2.0 * Math.PI);
        inputs.angleVelo = angleAbsoluteVeloSignal.getValue() * (2.0 * Math.PI);
        inputs.angleVolts = angleVoltSignal.getValue();
        inputs.angleAmps = angleAmpSignal.getValue();

        inputs.drivePosition = driveRotationsToMeters(drivePositionSignal.getValue());
        inputs.driveVelo = driveRotationsToMeters(driveVelocitySignal.getValue());
        inputs.driveVolts = driveVoltSignal.getValue();
        inputs.driveAmps = driveAmpSignal.getValue();


        Logger.processInputs("Swerve/SwerveModule[" + this.moduleNumber + "]", inputs);
    }
}
