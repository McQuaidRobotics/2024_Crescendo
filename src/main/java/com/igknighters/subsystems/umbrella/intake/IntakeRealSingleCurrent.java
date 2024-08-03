package com.igknighters.subsystems.umbrella.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.util.TunableValues;
import com.igknighters.util.TunableValues.TunableDouble;
import com.igknighters.util.can.CANSignalManager;

import edu.wpi.first.wpilibj.DriverStation;

import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import monologue.Annotations.Log;

public class IntakeRealSingleCurrent extends Intake {

    private final TalonFX motor = new TalonFX(kIntake.UPPER_MOTOR_ID, kUmbrella.CANBUS);
    private final StatusSignal<Double> voltSignal, ampSignal;
    @Log.NT
    private boolean forcedOutput = false;
    private TunableDouble currentTripValue = TunableValues.getDouble("IntakeCurrentTrip", 115.0);

    public IntakeRealSingleCurrent() {
        FaultManager.captureFault(
                UmbrellaHW.UpperIntakeMotor,
                motor.getConfigurator()
                        .apply(motorCfg()));

        voltSignal = motor.getMotorVoltage();
        ampSignal = motor.getTorqueCurrent();

        CANSignalManager.registerSignals(
            kUmbrella.CANBUS,
            voltSignal
        );
        ampSignal.setUpdateFrequency(200);

        motor.optimizeBusUtilization(1.0);

        BootupLogger.bootupLog("    Intake initialized (real)");
    }

    public TalonFXConfiguration motorCfg() {
        var cfg = new TalonFXConfiguration();

        cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = false;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentThreshold = 65.0;
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        cfg.CurrentLimits.SupplyTimeThreshold = 0.3;

        return cfg;
    }

    @Override
    public void setVoltageOut(double volts) {
        forcedOutput = false;
        super.voltsUpper = volts;
        if (super.exitBeamBroken) {
            motor.setVoltage(0.0);
        } else {
            motor.setVoltage(volts);
        }
    }

    @Override
    public void setVoltageOut(double volts, boolean force) {
        forcedOutput = force;
        if (!force) {
            setVoltageOut(volts);
            return;
        }
        super.voltsUpper = volts;
        motor.setVoltage(volts);
    }

    @Override
    public boolean isExitBeamBroken() {
        return super.exitBeamBroken;
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                UmbrellaHW.UpperIntakeMotor,
                    voltSignal,ampSignal);

        if (DriverStation.isDisabled()) {
            super.exitBeamBroken = false;
        }

        if (!super.exitBeamBroken && !forcedOutput) {
            ampSignal.refresh();
            super.exitBeamBroken = Math.abs(ampSignal.getValue()) > currentTripValue.get();
        } else if (super.exitBeamBroken && forcedOutput) {
            super.exitBeamBroken = false;
        }

        super.voltsUpper = voltSignal.getValue();
        super.ampsUpper = ampSignal.getValue();
        super.voltsLower = 0;
        super.ampsLower = 0;
    }
}
