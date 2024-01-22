package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;

import edu.wpi.first.math.util.Units;

public class ShooterReal implements Shooter {

    private final TalonFX motor = new TalonFX(kShooter.MOTOR_ID);
    private final ShooterInputs inputs = new ShooterInputs();
    private final StatusSignal<Double> veloSignal, voltSignal, currentSignal, tempSignal;

    public ShooterReal() {
        motor.getConfigurator().apply(motorConfig());

        veloSignal = motor.getVelocity();
        voltSignal = motor.getMotorVoltage();
        currentSignal = motor.getTorqueCurrent();
        tempSignal = motor.getDeviceTemp();

        veloSignal.setUpdateFrequency(100);
        voltSignal.setUpdateFrequency(100);
        currentSignal.setUpdateFrequency(100);
        tempSignal.setUpdateFrequency(4);

        motor.optimizeBusUtilization();
    }

    private TalonFXConfiguration motorConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_kP;
        cfg.Slot0.kI = kShooter.MOTOR_kI;
        cfg.Slot0.kD = kShooter.MOTOR_kD;
        return cfg;
    }

    @Override
    public double getSpeed() {
        return inputs.radiansPerSecond;
    }

    @Override
    public double getTargetSpeed() {
        return inputs.targetRadiansPerSecond;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        inputs.targetRadiansPerSecond = speedRadPerSec;

        var ret = new VelocityDutyCycle(Units.radiansToRotations(speedRadPerSec));
        motor.setControl(ret);
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.targetRadiansPerSecond = 0.0;
        inputs.volts = volts;
        motor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            veloSignal, voltSignal,
            currentSignal, tempSignal
        );

        inputs.amps = currentSignal.getValue();
        inputs.radiansPerSecond = Units.rotationsToRadians(veloSignal.getValue());
        inputs.volts = voltSignal.getValue();
        inputs.temp = tempSignal.getValue();

        Logger.processInputs("/Umbrella/Shooter", inputs);
    }
}
