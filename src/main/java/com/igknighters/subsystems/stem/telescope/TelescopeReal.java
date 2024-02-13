package com.igknighters.subsystems.stem.telescope;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.FaultManager;
import com.igknighters.util.SafeTalonFXConfiguration;


public class TelescopeReal implements Telescope {
    private final TalonFX motor;

    private final StatusSignal <Double> motorVolts, motorTemp, motorAmps, motorVelo, motorRots;
    private final StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final TelescopeInputs inputs;

    public TelescopeReal(){
        motor = new TalonFX(kTelescope.MOTOR_ID);
        motor.getConfigurator().apply(motorConfig());

        motorRots = motor.getRotorPosition();
        motorVelo = motor.getRotorVelocity();
        motorAmps = motor.getTorqueCurrent();
        motorVolts = motor.getMotorVoltage();
        motorTemp = motor.getDeviceTemp();

        motorRots.setUpdateFrequency(100);
        motorVelo.setUpdateFrequency(100);
        motorAmps.setUpdateFrequency(100);
        motorVolts.setUpdateFrequency(100);
        motorTemp.setUpdateFrequency(4);

        forwardLimitSwitch = motor.getForwardLimit();
        reverseLimitSwitch = motor.getReverseLimit();

        motor.optimizeBusUtilization();

        inputs = new TelescopeInputs(kTelescope.MIN_METERS);
    }

    private TalonFXConfiguration motorConfig() {
        TalonFXConfiguration telescopeMotorCfg = new SafeTalonFXConfiguration();
        telescopeMotorCfg.Slot0.kP = kTelescope.MOTOR_kP;
        telescopeMotorCfg.Slot0.kI = kTelescope.MOTOR_kI;
        telescopeMotorCfg.Slot0.kD = kTelescope.MOTOR_kD;

        telescopeMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        telescopeMotorCfg.MotorOutput.Inverted = kTelescope.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        telescopeMotorCfg.HardwareLimitSwitch.ReverseLimitEnable = true;
        telescopeMotorCfg.HardwareLimitSwitch.ForwardLimitEnable = true;

        telescopeMotorCfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        telescopeMotorCfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = mechMetersToMotorRots(kTelescope.MIN_METERS);

        telescopeMotorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        telescopeMotorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = mechMetersToMotorRots(kTelescope.MAX_METERS);

        return telescopeMotorCfg;
    }

    private double mechMetersToMotorRots(Double mechMeters){
        return ((mechMeters - kTelescope.MIN_METERS)
                / kTelescope.SPROCKET_CIRCUMFERENCE)
                * kTelescope.MOTOR_TO_MECHANISM_RATIO;
    }

    private double motorRotsToMechMeters(Double motorRots){
        return ((motorRots / kTelescope.MOTOR_TO_MECHANISM_RATIO)
                * kTelescope.SPROCKET_CIRCUMFERENCE)
                + kTelescope.MIN_METERS;
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.targetMeters = 0.0;
        this.motor.setVoltage(volts);
    }

    @Override
    public void setTelescopeMeters(double meters) {
        inputs.targetMeters = meters;
        var posControlRequest = new MotionMagicVoltage(mechMetersToMotorRots(meters));
        this.motor.setControl(posControlRequest);
    }

    @Override
    public double getTelescopeMeters() {
        return inputs.meters;
    }

    @Override
    public void stopMechanism(){
        inputs.volts = 0.0;
        this.motor.setVoltage(0);
    }

    @Override
    public boolean isFwdLimitSwitchHit() {
        return inputs.isLimitFwdSwitchHit;
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        return inputs.isLimitRevSwitchHit;
    }

    @Override
    public void periodic(){
        FaultManager.captureFault(
            StemHW.TelescopeMotor,
            BaseStatusSignal.refreshAll(
                motorRots, motorVelo,
                motorVolts, motorAmps,
                motorTemp, forwardLimitSwitch,
                reverseLimitSwitch));

        inputs.meters = motorRotsToMechMeters(motorRots.getValue());
        inputs.metersPerSecond = motorRotsToMechMeters(motorVelo.getValue());
        inputs.volts = motorVolts.getValue();
        inputs.temp = motorTemp.getValue();
        inputs.amps =  motorAmps.getValue();
        inputs.isLimitFwdSwitchHit = forwardLimitSwitch.getValue() == ForwardLimitValue.Open;
        inputs.isLimitRevSwitchHit = reverseLimitSwitch.getValue() == ReverseLimitValue.Open;

        Logger.processInputs("Stem/Telescope", inputs);
    }

}
