package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;

import edu.wpi.first.math.util.Units;

public class ShooterReal implements Shooter {

    private final TalonFX upperMotor = new TalonFX(kShooter.UPPER_MOTOR_ID);
    private final TalonFX lowerMotor = new TalonFX(kShooter.LOWER_MOTOR_ID);
    private final ShooterInputs inputs = new ShooterInputs();
    private final StatusSignal<Double> veloSignalUpper, voltSignalUpper, currentSignalUpper, tempSignalUpper;
    private final StatusSignal<Double> veloSignalLower, voltSignalLower, currentSignalLower, tempSignalLower;

    public ShooterReal() {
        upperMotor.getConfigurator().apply(motorUpperConfig());

        veloSignalUpper = upperMotor.getVelocity();
        voltSignalUpper = upperMotor.getMotorVoltage();
        currentSignalUpper = upperMotor.getTorqueCurrent();
        tempSignalUpper = upperMotor.getDeviceTemp();

        veloSignalUpper.setUpdateFrequency(100);
        voltSignalUpper.setUpdateFrequency(100);
        currentSignalUpper.setUpdateFrequency(100);
        tempSignalUpper.setUpdateFrequency(4);

        upperMotor.optimizeBusUtilization();

        lowerMotor.getConfigurator().apply(motorLowerConfig());

        veloSignalLower = lowerMotor.getVelocity();
        voltSignalLower = lowerMotor.getMotorVoltage();
        currentSignalLower = lowerMotor.getTorqueCurrent();
        tempSignalLower = lowerMotor.getDeviceTemp();

        veloSignalLower.setUpdateFrequency(100);
        voltSignalLower.setUpdateFrequency(100);
        currentSignalLower.setUpdateFrequency(100);
        tempSignalLower.setUpdateFrequency(4);

        lowerMotor.optimizeBusUtilization();
    }

    private TalonFXConfiguration motorUpperConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_UPPER_kP;
        cfg.Slot0.kI = kShooter.MOTOR_UPPER_kI;
        cfg.Slot0.kD = kShooter.MOTOR_UPPER_kD;
        cfg.Slot0.kS = kShooter.MOTOR_UPPER_kS;
        cfg.Slot0.kV = kShooter.MOTOR_UPPER_kV;

        cfg.CurrentLimits.StatorCurrentLimit = 100.0;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return cfg;
    }

    private TalonFXConfiguration motorLowerConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_LOWER_kP;
        cfg.Slot0.kI = kShooter.MOTOR_LOWER_kI;
        cfg.Slot0.kD = kShooter.MOTOR_LOWER_kD;
        cfg.Slot0.kS = kShooter.MOTOR_LOWER_kS;
        cfg.Slot0.kV = kShooter.MOTOR_LOWER_kV;

        cfg.CurrentLimits.StatorCurrentLimit = 100.0;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return cfg;
    }

    @Override
    public double getSpeed() {
        return (inputs.radiansPerSecondUpper + inputs.radiansPerSecondLower) / 2.0;
    }

    @Override
    public double getTargetSpeed() {
        return (inputs.targetRadiansPerSecondUpper + inputs.targetRadiansPerSecondLower) / 2.0;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        inputs.targetRadiansPerSecondUpper = speedRadPerSec;
        inputs.targetRadiansPerSecondLower = speedRadPerSec;
        var req = new VelocityDutyCycle(Units.radiansToRotations(speedRadPerSec));
        upperMotor.setControl(req);
        lowerMotor.setControl(req);
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.targetRadiansPerSecondUpper = 0.0;
        inputs.targetRadiansPerSecondLower = 0.0;
        inputs.voltsUpper = volts;
        inputs.voltsLower = volts;
        upperMotor.setVoltage(volts);
        lowerMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                veloSignalUpper,
                voltSignalUpper,
                currentSignalUpper,
                tempSignalUpper,
                veloSignalLower,
                voltSignalLower,
                currentSignalLower,
                tempSignalLower);

        inputs.radiansPerSecondUpper = Units.rotationsToRadians(veloSignalUpper.getValue());
        inputs.voltsUpper = voltSignalUpper.getValue();
        inputs.ampsUpper = currentSignalUpper.getValue();
        inputs.tempUpper = tempSignalUpper.getValue();

        inputs.radiansPerSecondLower = Units.rotationsToRadians(veloSignalLower.getValue());
        inputs.voltsLower = voltSignalLower.getValue();
        inputs.ampsLower = currentSignalLower.getValue();
        inputs.tempLower = tempSignalLower.getValue();


        Logger.processInputs("/Umbrella/Shooter", inputs);
    }
}
