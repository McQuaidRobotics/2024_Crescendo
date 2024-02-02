package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import edu.wpi.first.math.util.Units;

public class ShooterReal implements Shooter {

    private final TalonFX rightMotor = new TalonFX(kShooter.RIGHT_MOTOR_ID);
    private final TalonFX leftMotor = new TalonFX(kShooter.LEFT_MOTOR_ID);
    private final ShooterInputs inputs = new ShooterInputs();
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
        cfg.Slot0.kP = kShooter.MOTOR_UPPER_kP;
        cfg.Slot0.kI = kShooter.MOTOR_UPPER_kI;
        cfg.Slot0.kD = kShooter.MOTOR_UPPER_kD;
        cfg.Slot0.kS = kShooter.MOTOR_UPPER_kS;
        cfg.Slot0.kV = kShooter.MOTOR_UPPER_kV;

        cfg.CurrentLimits.StatorCurrentLimit = kShooter.PEAK_CURRENT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return cfg;
    }

    private TalonFXConfiguration motorLeftConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_LOWER_kP;
        cfg.Slot0.kI = kShooter.MOTOR_LOWER_kI;
        cfg.Slot0.kD = kShooter.MOTOR_LOWER_kD;
        cfg.Slot0.kS = kShooter.MOTOR_LOWER_kS;
        cfg.Slot0.kV = kShooter.MOTOR_LOWER_kV;

        cfg.CurrentLimits.StatorCurrentLimit = kShooter.PEAK_CURRENT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return cfg;
    }

    @Override
    public double getSpeed() {
        return (inputs.radiansPerSecondRight + inputs.radiansPerSecondLeft) / 2.0;
    }

    @Override
    public double getTargetSpeed() {
        return (inputs.targetRadiansPerSecondRight + inputs.targetRadiansPerSecondLeft) / 2.0;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        inputs.targetRadiansPerSecondRight = speedRadPerSec;
        inputs.targetRadiansPerSecondLeft = speedRadPerSec;
        var req = new VelocityDutyCycle(Units.radiansToRotations(speedRadPerSec));
        rightMotor.setControl(req);
        leftMotor.setControl(req);
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.targetRadiansPerSecondRight = 0.0;
        inputs.targetRadiansPerSecondLeft = 0.0;
        inputs.voltsRight = volts;
        inputs.voltsLeft = volts;
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

        inputs.radiansPerSecondRight = Units.rotationsToRadians(veloSignalRight.getValue());
        inputs.voltsRight = voltSignalRight.getValue();
        inputs.ampsRight = currentSignalRight.getValue();
        inputs.tempRight = tempSignalRight.getValue();

        inputs.radiansPerSecondLeft = Units.rotationsToRadians(veloSignalLeft.getValue());
        inputs.voltsLeft = voltSignalLeft.getValue();
        inputs.ampsLeft = currentSignalLeft.getValue();
        inputs.tempLeft = tempSignalLeft.getValue();

        Logger.processInputs("/Umbrella/Shooter", inputs);
    }
}
