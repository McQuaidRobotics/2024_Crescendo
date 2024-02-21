package com.igknighters.subsystems.stem.telescope;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.FaultManager;
import com.igknighters.util.SafeTalonFXConfiguration;


public class TelescopeReal implements Telescope {
    private final TalonFX motor;

    private final StatusSignal<Double> motorVolts, motorTemp, motorAmps, motorVelo, motorRots;
    private final StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final TelescopeInputs inputs;

    private boolean hasHomed = false;

    public TelescopeReal(){
        motor = new TalonFX(kTelescope.MOTOR_ID, kStem.CANBUS);
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
        TalonFXConfiguration cfg = new SafeTalonFXConfiguration();
        cfg.Slot0.kP = kTelescope.MOTOR_kP;
        cfg.Slot0.kI = kTelescope.MOTOR_kI;
        cfg.Slot0.kD = kTelescope.MOTOR_kD;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kTelescope.MAX_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = kTelescope.MAX_ACCELERATION;
        cfg.MotionMagic.MotionMagicJerk = kTelescope.MAX_JERK;



        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = kTelescope.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
        cfg.HardwareLimitSwitch.ForwardLimitEnable = true;

        cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = mechMetersToMotorRots(kTelescope.MAX_METERS);

        cfg.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;

        cfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        cfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = mechMetersToMotorRots(kTelescope.MIN_METERS);

        cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

        return cfg;
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
    public boolean hasHomed() {
        return hasHomed;
    }

    @Override
    public void setCoast(boolean shouldBeCoasting) {
        this.motor.setNeutralMode(
            shouldBeCoasting
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake
        );
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

        if (!hasHomed && (inputs.isLimitFwdSwitchHit || inputs.isLimitRevSwitchHit)){
            hasHomed = true;
        }

        Logger.processInputs("Stem/Telescope", inputs);
    }

}
