package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import edu.wpi.first.math.util.Units;

public class IntakeReal implements Intake {

    private final TalonFX upperMotor = new TalonFX(kIntake.UPPER_MOTOR_ID);
    private final TalonFX lowerMotor = new TalonFX(kIntake.LOWER_MOTOR_ID);
    private final StatusSignal<Double> veloSignalUpper, voltSignalUpper, currentSignalUpper, tempSignalUpper;
    private final StatusSignal<Double> veloSignalLower, voltSignalLower, currentSignalLower, tempSignalLower;
    private final StatusSignal<ReverseLimitValue> revLimitSignal;
    private final IntakeInputs inputs = new IntakeInputs();
    private final HardwareLimitSwitchConfigs lowerLimitCfg, upperLimitCfg;
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
        tempSignalUpper = upperMotor.getDeviceTemp();

        veloSignalUpper.setUpdateFrequency(100);
        voltSignalUpper.setUpdateFrequency(100);
        currentSignalUpper.setUpdateFrequency(100);
        tempSignalUpper.setUpdateFrequency(100);

        veloSignalLower = lowerMotor.getVelocity();
        voltSignalLower = lowerMotor.getMotorVoltage();
        currentSignalLower = lowerMotor.getTorqueCurrent();
        tempSignalLower = lowerMotor.getDeviceTemp();

        veloSignalLower.setUpdateFrequency(100);
        voltSignalLower.setUpdateFrequency(100);
        currentSignalLower.setUpdateFrequency(100);
        tempSignalLower.setUpdateFrequency(100);

        if (kIntake.BEAM_IS_UPPER) {
            revLimitSignal = upperMotor.getReverseLimit();
        } else {
            revLimitSignal = lowerMotor.getReverseLimit();
        }

        revLimitSignal.setUpdateFrequency(250);

        upperMotor.optimizeBusUtilization();
        lowerMotor.optimizeBusUtilization();

        BootupLogger.bootupLog("    Intake initialized (real)");
    }

    public TalonFXConfiguration motorUpper() {
        var cfg = new TalonFXConfiguration();

        cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = false;

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
        cfg.HardwareLimitSwitch.ReverseLimitEnable = false;

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
        inputs.voltsLower = volts;
        inputs.voltsUpper = volts * kIntake.UPPER_DIFF;
        if (wasBeamBroken) {
            lowerMotor.setVoltage(0.0);
            upperMotor.setVoltage(0.0);
        } else {
            lowerMotor.setVoltage(volts);
            upperMotor.setVoltage(volts * kIntake.UPPER_DIFF);
        }
    }

    @Override
    public void turnIntakeRads(double radians) {
        setVoltageOut(0.0);
        upperMotor.setControl(new PositionVoltage(
                upperMotor.getRotorPosition().getValue()
                        + Units.radiansToRotations(radians * kIntake.UPPER_DIFF)));
        lowerMotor.setControl(new PositionVoltage(
                lowerMotor.getRotorPosition().getValue()
                        + Units.radiansToRotations(radians)));
    }

    @Override
    public boolean isExitBeamBroken() {
        return inputs.exitBeamBroken;
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                UmbrellaHW.IntakeMotor,
                BaseStatusSignal.refreshAll(
                        veloSignalUpper, voltSignalUpper,
                        currentSignalUpper, tempSignalUpper));

        inputs.exitBeamBroken = revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
        inputs.radiansPerSecondUpper = Units.rotationsToRadians(veloSignalUpper.getValue());
        inputs.voltsUpper = voltSignalUpper.getValue();
        inputs.ampsUpper = currentSignalUpper.getValue();
        inputs.tempUpper = tempSignalUpper.getValue();
        inputs.radiansPerSecondLower = Units.rotationsToRadians(veloSignalLower.getValue());
        inputs.voltsLower = voltSignalLower.getValue();
        inputs.ampsLower = currentSignalLower.getValue();
        inputs.tempLower = tempSignalLower.getValue();

        if (inputs.exitBeamBroken && !wasBeamBroken) {
            lowerMotor.getConfigurator().apply(lowerLimitCfg.withReverseLimitEnable(false));
            upperMotor.getConfigurator().apply(upperLimitCfg.withReverseLimitEnable(false));
            wasBeamBroken = true;
        } else if (!inputs.exitBeamBroken && wasBeamBroken) {
            lowerMotor.getConfigurator().apply(lowerLimitCfg.withReverseLimitEnable(true));
            upperMotor.getConfigurator().apply(upperLimitCfg.withReverseLimitEnable(true));
            wasBeamBroken = false;
        }

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
