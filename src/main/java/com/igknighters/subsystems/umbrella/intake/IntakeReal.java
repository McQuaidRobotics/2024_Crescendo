package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;

import edu.wpi.first.math.util.Units;

public class IntakeReal implements Intake {

    private final TalonFX motor = new TalonFX(kIntake.MOTOR_ID);
    private final StatusSignal<Double> veloSignal, voltSignal, currentSignal, tempSignal;
    private final StatusSignal<ReverseLimitValue> revLimitSignal;
    private final StatusSignal<ForwardLimitValue> fwdLimitSignal;
    private final IntakeInputs inputs = new IntakeInputs();

    public IntakeReal() {
        motor.getConfigurator().apply(new TalonFXConfiguration());

        veloSignal = motor.getVelocity();
        voltSignal = motor.getMotorVoltage();
        currentSignal = motor.getTorqueCurrent();
        tempSignal = motor.getDeviceTemp();
        revLimitSignal = motor.getReverseLimit();
        fwdLimitSignal = motor.getForwardLimit();

        veloSignal.setUpdateFrequency(100);
        voltSignal.setUpdateFrequency(100);
        currentSignal.setUpdateFrequency(100);
        tempSignal.setUpdateFrequency(100);
        revLimitSignal.setUpdateFrequency(100);
        fwdLimitSignal.setUpdateFrequency(100);

        motor.optimizeBusUtilization();
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.volts = volts;
        motor.setVoltage(volts);
    }

    @Override
    public void turnIntakeRads(double radians) {
        setVoltageOut(0.0);
        var ret = new PositionDutyCycle(
                motor.getRotorPosition().getValue() + Units.radiansToRotations(radians));
        motor.setControl(ret);
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
        BaseStatusSignal.refreshAll(
                veloSignal, voltSignal,
                currentSignal, tempSignal);

        inputs.entranceBeamBroken = revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
        inputs.exitBeamBroken = fwdLimitSignal.getValue().equals(ForwardLimitValue.ClosedToGround);
        inputs.radiansPerSecond = Units.rotationsToRadians(veloSignal.getValue());
        inputs.volts = voltSignal.getValue();
        inputs.amps = currentSignal.getValue();
        inputs.temp = tempSignal.getValue();

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
