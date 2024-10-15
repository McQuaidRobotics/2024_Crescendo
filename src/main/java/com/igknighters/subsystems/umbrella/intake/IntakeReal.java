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

public class IntakeReal extends Intake {

    private final TalonFX upperMotor = new TalonFX(kIntake.UPPER_MOTOR_ID, kUmbrella.CANBUS);
    private final TalonFX lowerMotor = new TalonFX(kIntake.LOWER_MOTOR_ID, kUmbrella.CANBUS);

    private final StatusSignal<Double> voltUpperSignal, ampUpperSignal;
    private final StatusSignal<Double> voltLowerSignal, ampLowerSignal;
    private final StatusSignal<ReverseLimitValue> revLimitSignal;

    private final HardwareLimitSwitchConfigs lowerLimitCfg, upperLimitCfg;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);

    @Log private boolean wasBeamBroken = false;

    public IntakeReal() {
        FaultManager.captureFault(
                UmbrellaHW.UpperIntakeMotor,
                upperMotor.getConfigurator()
                        .apply(motorUpper()));
        FaultManager.captureFault(
                UmbrellaHW.LowerIntakeMotor,
                lowerMotor.getConfigurator()
                        .apply(motorLower()));

        var lowerLimitCfg = new HardwareLimitSwitchConfigs();
        lowerMotor.getConfigurator().refresh(lowerLimitCfg);
        this.lowerLimitCfg = lowerLimitCfg;

        var upperLimitCfg = new HardwareLimitSwitchConfigs();
        upperMotor.getConfigurator().refresh(upperLimitCfg);
        this.upperLimitCfg = upperLimitCfg;

        voltUpperSignal = upperMotor.getMotorVoltage();
        ampUpperSignal = upperMotor.getTorqueCurrent();

        voltLowerSignal = lowerMotor.getMotorVoltage();
        ampLowerSignal = lowerMotor.getTorqueCurrent();

        CANSignalManager.registerSignals(
            kUmbrella.CANBUS,
            voltUpperSignal, ampUpperSignal,
            voltLowerSignal, ampLowerSignal
        );


        if (kIntake.BEAM_IS_UPPER) {
            revLimitSignal = upperMotor.getReverseLimit();
        } else {
            revLimitSignal = lowerMotor.getReverseLimit();
        }

        revLimitSignal.setUpdateFrequency(250);

        upperMotor.optimizeBusUtilization(1.0);
        lowerMotor.optimizeBusUtilization(1.0);

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
        super.voltsLower = volts;
        super.voltsUpper = volts * kIntake.UPPER_DIFF;
        if (super.exitBeamBroken) {
            lowerMotor.setControl(controlReqVolts.withOutput(0.0));
            upperMotor.setControl(controlReqVolts.withOutput(0.0));
        } else {
            lowerMotor.setControl(controlReqVolts.withOutput(volts));
            upperMotor.setControl(controlReqVolts.withOutput(volts * kIntake.UPPER_DIFF));
        }
    }

    @Override
    public void setVoltageOut(double volts, boolean force) {
        if (!force) {
            setVoltageOut(volts);
            return;
        }
        super.voltsLower = volts;
        super.voltsUpper = volts * kIntake.UPPER_DIFF;
        lowerMotor.setControl(controlReqVolts.withOutput(volts));
        upperMotor.setControl(controlReqVolts.withOutput(volts * kIntake.UPPER_DIFF));
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

        FaultManager.captureFault(
                UmbrellaHW.LowerIntakeMotor,
                    voltLowerSignal,ampLowerSignal);

        //This requires as little latency as possible so refresh on its own closer to control code
        revLimitSignal.refresh();

        super.exitBeamBroken = revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
        super.voltsUpper = voltUpperSignal.getValueAsDouble();
        super.ampsUpper = ampUpperSignal.getValueAsDouble();
        super.voltsLower = voltLowerSignal.getValueAsDouble();
        super.ampsLower = ampLowerSignal.getValueAsDouble();

        if (super.exitBeamBroken && !wasBeamBroken) {
            this.setVoltageOut(0.0);
            lowerMotor.getConfigurator().apply(lowerLimitCfg.withReverseLimitEnable(false));
            upperMotor.getConfigurator().apply(upperLimitCfg.withReverseLimitEnable(false));
            wasBeamBroken = true;
        } else if (!super.exitBeamBroken && wasBeamBroken) {
            lowerMotor.getConfigurator().apply(lowerLimitCfg.withReverseLimitEnable(true));
            upperMotor.getConfigurator().apply(upperLimitCfg.withReverseLimitEnable(true));
            wasBeamBroken = false;
        }
    }
}
