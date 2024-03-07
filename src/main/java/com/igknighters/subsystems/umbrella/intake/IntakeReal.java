package com.igknighters.subsystems.umbrella.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import edu.wpi.first.math.util.Units;
import monologue.Annotations.Log;

public class IntakeReal extends Intake {

    private final TalonFX upperMotor = new TalonFX(kIntake.UPPER_MOTOR_ID, kUmbrella.CANBUS);
    private final TalonFX lowerMotor = new TalonFX(kIntake.LOWER_MOTOR_ID, kUmbrella.CANBUS);
    private final StatusSignal<Double> veloSignalUpper, voltSignalUpper, currentSignalUpper;
    private final StatusSignal<Double> veloSignalLower, voltSignalLower, currentSignalLower;
    private final StatusSignal<ReverseLimitValue> revLimitSignal;
    private final HardwareLimitSwitchConfigs lowerLimitCfg, upperLimitCfg;
    @Log.NT
    private boolean wasBeamBroken = false;

    public IntakeReal() {
        FaultManager.captureFault(
                UmbrellaHW.IntakeMotor,
                upperMotor.getConfigurator()
                        .apply(motorUpper()));
        FaultManager.captureFault(
                UmbrellaHW.IntakeMotor,
                lowerMotor.getConfigurator()
                        .apply(motorLower()));

        var lowerLimitCfg = new HardwareLimitSwitchConfigs();
        lowerMotor.getConfigurator().refresh(lowerLimitCfg);
        this.lowerLimitCfg = lowerLimitCfg;

        var upperLimitCfg = new HardwareLimitSwitchConfigs();
        upperMotor.getConfigurator().refresh(upperLimitCfg);
        this.upperLimitCfg = upperLimitCfg;

        veloSignalUpper = upperMotor.getVelocity();
        voltSignalUpper = upperMotor.getMotorVoltage();
        currentSignalUpper = upperMotor.getTorqueCurrent();

        veloSignalUpper.setUpdateFrequency(100);
        voltSignalUpper.setUpdateFrequency(100);
        currentSignalUpper.setUpdateFrequency(100);

        veloSignalLower = lowerMotor.getVelocity();
        voltSignalLower = lowerMotor.getMotorVoltage();
        currentSignalLower = lowerMotor.getTorqueCurrent();

        veloSignalLower.setUpdateFrequency(100);
        voltSignalLower.setUpdateFrequency(100);
        currentSignalLower.setUpdateFrequency(100);

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
            lowerMotor.setVoltage(0.0);
            upperMotor.setVoltage(0.0);
        } else {
            lowerMotor.setVoltage(volts);
            upperMotor.setVoltage(volts * kIntake.UPPER_DIFF);
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
        lowerMotor.setVoltage(volts);
        upperMotor.setVoltage(volts * kIntake.UPPER_DIFF);
    }

    @Override
    public boolean isExitBeamBroken() {
        return super.exitBeamBroken;
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                UmbrellaHW.IntakeMotor,
                BaseStatusSignal.refreshAll(
                        veloSignalUpper, voltSignalUpper,
                        currentSignalUpper, revLimitSignal));

        super.exitBeamBroken = revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
        super.radiansPerSecondUpper = Units.rotationsToRadians(veloSignalUpper.getValue());
        super.voltsUpper = voltSignalUpper.getValue();
        super.ampsUpper = currentSignalUpper.getValue();
        super.radiansPerSecondLower = Units.rotationsToRadians(veloSignalLower.getValue());
        super.voltsLower = voltSignalLower.getValue();
        super.ampsLower = currentSignalLower.getValue();

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
