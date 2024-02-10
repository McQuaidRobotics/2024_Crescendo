package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
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
    private final StatusSignal<ForwardLimitValue> fwdLimitSignal;
    private final IntakeInputs inputs = new IntakeInputs();

    public IntakeReal() {
        FaultManager.captureFault(
                UmbrellaHW.IntakeMotor,
                upperMotor.getConfigurator()
                        .apply(new TalonFXConfiguration()));
        FaultManager.captureFault(
                UmbrellaHW.IntakeMotor,
                lowerMotor.getConfigurator()
                        .apply(new TalonFXConfiguration()));

        upperMotor.getConfigurator().apply(motorUpperFollower());
        lowerMotor.getConfigurator().apply(motorLowerUpper());

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
            fwdLimitSignal = upperMotor.getForwardLimit();
        } else {
            revLimitSignal = lowerMotor.getReverseLimit();
            fwdLimitSignal = lowerMotor.getForwardLimit();
        }

        revLimitSignal.setUpdateFrequency(250);
        fwdLimitSignal.setUpdateFrequency(250);

        upperMotor.optimizeBusUtilization();
        lowerMotor.optimizeBusUtilization();

        BootupLogger.bootupLog("    Intake initialized (real)");
    }

    public TalonFXConfiguration motorUpperFollower() {
        var cfg = new TalonFXConfiguration();

        cfg.HardwareLimitSwitch.ForwardLimitEnable = true;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

        cfg.HardwareLimitSwitch.ForwardLimitRemoteSensorID = kIntake.LOWER_MOTOR_ID;
        cfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = kIntake.LOWER_MOTOR_ID;

        cfg.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
        cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 80.0;

        return cfg;
    }

    public TalonFXConfiguration motorLowerUpper() {
        var cfg = new TalonFXConfiguration();

        cfg.HardwareLimitSwitch.ForwardLimitEnable = true;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 80.0;

        return cfg;
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.voltsLower = volts;
        inputs.voltsUpper = volts * kIntake.UPPER_DIFF;
        lowerMotor.setVoltage(volts);
        upperMotor.setVoltage(volts * kIntake.UPPER_DIFF);
    }

    @Override
    public void turnIntakeRads(double radians) {
        setVoltageOut(0.0);
        upperMotor.setControl(new PositionDutyCycle(
                upperMotor.getRotorPosition().getValue()
                        + Units.radiansToRotations(radians * kIntake.UPPER_DIFF)));
        lowerMotor.setControl(new PositionDutyCycle(
                lowerMotor.getRotorPosition().getValue()
                        + Units.radiansToRotations(radians)));
    }

    @Override
    public boolean isEntranceBeamBroken() {
        return inputs.entranceBeamBroken;
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

        inputs.entranceBeamBroken = revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
        inputs.exitBeamBroken = fwdLimitSignal.getValue().equals(ForwardLimitValue.ClosedToGround);
        inputs.radiansPerSecondUpper = Units.rotationsToRadians(veloSignalUpper.getValue());
        inputs.voltsUpper = voltSignalUpper.getValue();
        inputs.ampsUpper = currentSignalUpper.getValue();
        inputs.tempUpper = tempSignalUpper.getValue();
        inputs.radiansPerSecondLower = Units.rotationsToRadians(veloSignalLower.getValue());
        inputs.voltsLower = voltSignalLower.getValue();
        inputs.ampsLower = currentSignalLower.getValue();
        inputs.tempLower = tempSignalLower.getValue();

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
