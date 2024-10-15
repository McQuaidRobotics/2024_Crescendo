package com.igknighters.subsystems.umbrella.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.util.logging.FaultManager;
import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import monologue.Annotations.Log;

public class IntakeRealSingle extends Intake {

    private final TalonFX upperMotor = new TalonFX(kIntake.UPPER_MOTOR_ID, kUmbrella.CANBUS);

    private final StatusSignal<Double> voltUpperSignal, ampUpperSignal;
    private final StatusSignal<ReverseLimitValue> revLimitSignal;

    private final HardwareLimitSwitchConfigs upperLimitCfg;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);

    @Log private boolean wasBeamBroken = false;

    public IntakeRealSingle() {
        FaultManager.captureFault(
                UmbrellaHW.UpperIntakeMotor,
                upperMotor.getConfigurator()
                        .apply(motorUpper()));

        var upperLimitCfg = new HardwareLimitSwitchConfigs();
        upperMotor.getConfigurator().refresh(upperLimitCfg);
        this.upperLimitCfg = upperLimitCfg;

        voltUpperSignal = upperMotor.getMotorVoltage();
        ampUpperSignal = upperMotor.getTorqueCurrent();

        CANSignalManager.registerSignals(
            kUmbrella.CANBUS,
            voltUpperSignal, ampUpperSignal
        );


        revLimitSignal = upperMotor.getReverseLimit();

        revLimitSignal.setUpdateFrequency(250);

        upperMotor.optimizeBusUtilization(1.0);

        BootupLogger.bootupLog("    Intake initialized (real)");
    }

    public TalonFXConfiguration motorUpper() {
        var cfg = new TalonFXConfiguration();

        cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentThreshold = 65.0;
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        cfg.CurrentLimits.SupplyTimeThreshold = 0.3;

        cfg.Audio.BeepOnConfig = false;

        return cfg;
    }

    public TalonFXConfiguration motorLower() {
        var cfg = new TalonFXConfiguration();

        cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

        cfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = kIntake.UPPER_MOTOR_ID;

        cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentThreshold = 65.0;
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        cfg.CurrentLimits.SupplyTimeThreshold = 0.3;

        cfg.Audio.BeepOnConfig = false;

        return cfg;
    }

    @Override
    public void setVoltageOut(double volts) {
        super.voltsUpper = volts;
        if (super.exitBeamBroken) {
            upperMotor.setControl(controlReqVolts.withOutput(0.0));
        } else {
            upperMotor.setControl(controlReqVolts.withOutput(volts));
        }
    }

    @Override
    public void setVoltageOut(double volts, boolean force) {
        if (!force) {
            setVoltageOut(volts);
            return;
        }
        super.voltsUpper = volts;
        upperMotor.setControl(controlReqVolts.withOutput(volts));
    }

    @Override
    public boolean isExitBeamBroken() {
        return super.exitBeamBroken;
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                UmbrellaHW.UpperIntakeMotor,
                    voltUpperSignal,ampUpperSignal);

        //This requires as little latency as possible so refresh on its own closer to control code
        revLimitSignal.refresh();

        super.exitBeamBroken = revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
        super.voltsUpper = voltUpperSignal.getValueAsDouble();
        super.ampsUpper = ampUpperSignal.getValueAsDouble();
        super.voltsLower = 0;
        super.ampsLower = 0;

        if (super.exitBeamBroken && !wasBeamBroken) {
            this.setVoltageOut(0.0);
            upperMotor.getConfigurator().apply(upperLimitCfg.withReverseLimitEnable(false));
            wasBeamBroken = true;
        } else if (!super.exitBeamBroken && wasBeamBroken) {
            upperMotor.getConfigurator().apply(upperLimitCfg.withReverseLimitEnable(true));
            wasBeamBroken = false;
        }
    }
}
