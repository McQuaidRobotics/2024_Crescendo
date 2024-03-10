package com.igknighters.subsystems.umbrella.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import edu.wpi.first.math.util.Units;

public class ShooterReal extends Shooter {

    private final TalonFX rightMotor = new TalonFX(kShooter.RIGHT_MOTOR_ID, kUmbrella.CANBUS);
    private final TalonFX leftMotor = new TalonFX(kShooter.LEFT_MOTOR_ID, kUmbrella.CANBUS);
    private final StatusSignal<Double> veloSignalRight, voltSignalRight, currentSignalRight, tempSignalRight;
    private final StatusSignal<Double> veloSignalLeft, voltSignalLeft, currentSignalLeft, tempSignalLeft;

    public ShooterReal() {
        rightMotor.getConfigurator().apply(motorRightConfig());

        veloSignalRight = rightMotor.getVelocity();
        voltSignalRight = rightMotor.getMotorVoltage();
        currentSignalRight = rightMotor.getTorqueCurrent();
        tempSignalRight = rightMotor.getDeviceTemp();

        veloSignalRight.setUpdateFrequency(100);
        voltSignalRight.setUpdateFrequency(100);
        currentSignalRight.setUpdateFrequency(100);
        tempSignalRight.setUpdateFrequency(4);

        rightMotor.optimizeBusUtilization();

        leftMotor.getConfigurator().apply(motorLeftConfig());

        veloSignalLeft = leftMotor.getVelocity();
        voltSignalLeft = leftMotor.getMotorVoltage();
        currentSignalLeft = leftMotor.getTorqueCurrent();
        tempSignalLeft = leftMotor.getDeviceTemp();

        veloSignalLeft.setUpdateFrequency(100);
        voltSignalLeft.setUpdateFrequency(100);
        currentSignalLeft.setUpdateFrequency(100);
        tempSignalLeft.setUpdateFrequency(4);

        leftMotor.optimizeBusUtilization();

        BootupLogger.bootupLog("    Shooter initialized (real)");
    }

    private TalonFXConfiguration motorRightConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_RIGHT_kP;
        cfg.Slot0.kI = kShooter.MOTOR_RIGHT_kI;
        cfg.Slot0.kD = kShooter.MOTOR_RIGHT_kD;
        cfg.Slot0.kS = kShooter.MOTOR_RIGHT_kS;
        cfg.Slot0.kV = kShooter.MOTOR_RIGHT_kV;

        cfg.CurrentLimits.StatorCurrentLimit = kShooter.PEAK_CURRENT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;
        cfg.Voltage.PeakReverseVoltage = 0.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;


        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return cfg;
    }

    private TalonFXConfiguration motorLeftConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_LEFT_kP;
        cfg.Slot0.kI = kShooter.MOTOR_LEFT_kI;
        cfg.Slot0.kD = kShooter.MOTOR_LEFT_kD;
        cfg.Slot0.kS = kShooter.MOTOR_LEFT_kS;
        cfg.Slot0.kV = kShooter.MOTOR_LEFT_kV;

        cfg.CurrentLimits.StatorCurrentLimit = kShooter.PEAK_CURRENT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;
        cfg.Voltage.PeakReverseVoltage = 0.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return cfg;
    }

    @Override
    public double getSpeed() {
        return (super.radiansPerSecondRight + super.radiansPerSecondLeft) / 2.0;
    }

    @Override
    public double getTargetSpeed() {
        return (super.targetRadiansPerSecondRight + super.targetRadiansPerSecondLeft) / 2.0;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        speedRadPerSec = Math.min(speedRadPerSec, kShooter.MAX_SHOOT_SPEED);

        super.targetRadiansPerSecondRight = speedRadPerSec / kShooter.MECHANISM_RATIO;
        super.targetRadiansPerSecondLeft = (speedRadPerSec * kShooter.LEFT_MOTOR_DIFF);
        rightMotor.setControl(new VelocityVoltage(Units.radiansToRotations(super.targetRadiansPerSecondRight)));
        leftMotor.setControl(new VelocityVoltage(Units.radiansToRotations(super.targetRadiansPerSecondLeft)));
    }

    @Override
    public void setVoltageOut(double volts) {
        super.targetRadiansPerSecondRight = 0.0;
        super.targetRadiansPerSecondLeft = 0.0;
        super.voltsRight = volts;
        super.voltsLeft = volts;
        rightMotor.setVoltage(volts);
        leftMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                UmbrellaHW.RightShooterMotor,
                BaseStatusSignal.refreshAll(
                        veloSignalRight,
                        voltSignalRight,
                        currentSignalRight,
                        tempSignalRight));

        FaultManager.captureFault(
                UmbrellaHW.LeftShooterMotor,
                BaseStatusSignal.refreshAll(
                        veloSignalLeft,
                        voltSignalLeft,
                        currentSignalLeft,
                        tempSignalLeft));

        super.radiansPerSecondRight = Units.rotationsToRadians(veloSignalRight.getValue()) * kShooter.MECHANISM_RATIO;
        super.voltsRight = voltSignalRight.getValue();
        super.ampsRight = currentSignalRight.getValue();
        super.tempRight = tempSignalRight.getValue();

        super.radiansPerSecondLeft = Units.rotationsToRadians(veloSignalLeft.getValue()) * kShooter.MECHANISM_RATIO;
        super.voltsLeft = voltSignalLeft.getValue();
        super.ampsLeft = currentSignalLeft.getValue();
        super.tempLeft = tempSignalLeft.getValue();

        super.shooterRightRPM = Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondRight);
        super.shooterLeftRPM = Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondLeft);
    }
}
