package com.igknighters.subsystems.umbrella.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.util.logging.FaultManager;
import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableDouble;

import edu.wpi.first.wpilibj.DriverStation;

import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import monologue.Annotations.Log;

/**
 * 
 */
public class IntakeRealSingleCurrent extends Intake {

    private final TalonFX motor = new TalonFX(kIntake.UPPER_MOTOR_ID, kUmbrella.CANBUS);

    private final StatusSignal<Double> voltSignal, ampSignal;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);

    private TunableDouble currentTripValue = TunableValues.getDouble("IntakeCurrentTrip", 115.0);

    @Log private boolean forcedOutput = false;

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
            motor.setControl(controlReqVolts.withOutput(0.0));
        } else {
            motor.setControl(controlReqVolts.withOutput(volts));
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
        motor.setControl(controlReqVolts.withOutput(volts));
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
            super.exitBeamBroken = Math.abs(ampSignal.getValueAsDouble()) > currentTripValue.value();
        } else if (super.exitBeamBroken && forcedOutput) {
            super.exitBeamBroken = false;
        }

        super.voltsUpper = voltSignal.getValueAsDouble();
        super.ampsUpper = ampSignal.getValueAsDouble();
        super.voltsLower = 0;
        super.ampsLower = 0;
    }
}
