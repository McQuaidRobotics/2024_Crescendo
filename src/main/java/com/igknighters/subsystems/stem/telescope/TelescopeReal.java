package com.igknighters.subsystems.stem.telescope;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;

import monologue.Annotations.Log;

public class TelescopeReal extends Telescope {
    private final TalonFX motor;

    private final StatusSignal<Double> motorVolts, motorAmps, motorVelo, motorRots;
    private final StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    @Log.NT
    private boolean hasHomed = false;
    @Log.NT
    private boolean motorAutoseed = true;

    public TelescopeReal() {
        super(kTelescope.MIN_METERS);

        motor = new TalonFX(kTelescope.MOTOR_ID, kStem.CANBUS);
        CANRetrier.retryStatusCode(() -> motor.getConfigurator().apply(motorConfig(), 1.0), 5);

        motorRots = motor.getRotorPosition();
        motorVelo = motor.getRotorVelocity();
        motorAmps = motor.getTorqueCurrent();
        motorVolts = motor.getMotorVoltage();

        forwardLimitSwitch = motor.getForwardLimit();
        reverseLimitSwitch = motor.getReverseLimit();

        CANSignalManager.registerSignals(
                kStem.CANBUS,
                motorRots, motorVelo, motorVolts, motorAmps,
                forwardLimitSwitch, reverseLimitSwitch);

        motor.optimizeBusUtilization(1.0);
    }

    private TalonFXConfiguration motorConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
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

        cfg.TorqueCurrent.TorqueNeutralDeadband = 4.0;

        return cfg;
    }

    private double mechMetersToMotorRots(Double mechMeters) {
        return ((mechMeters - kTelescope.MIN_METERS)
                / kTelescope.SPROCKET_CIRCUMFERENCE)
                * kTelescope.MOTOR_TO_MECHANISM_RATIO;
    }

    private double motorRotsToMechMeters(Double motorRots) {
        return ((motorRots / kTelescope.MOTOR_TO_MECHANISM_RATIO)
                * kTelescope.SPROCKET_CIRCUMFERENCE)
                + kTelescope.MIN_METERS;
    }

    @Override
    public void setVoltageOut(double volts) {
        super.targetMeters = 0.0;
        this.motor.setVoltage(volts);
    }

    @Override
    public void setTelescopeMeters(double meters) {
        super.targetMeters = meters;
        var posControlRequest = new MotionMagicTorqueCurrentFOC(mechMetersToMotorRots(meters));
        this.motor.setControl(posControlRequest);
    }

    @Override
    public double getTelescopeMeters() {
        return super.meters;
    }

    @Override
    public void stopMechanism() {
        super.volts = 0.0;
        this.motor.setVoltage(0);
    }

    @Override
    public boolean isFwdLimitSwitchHit() {
        return super.isLimitFwdSwitchHit;
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        return super.isLimitRevSwitchHit;
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
                        : NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                StemHW.TelescopeMotor,
                motorRots, motorVelo,
                motorVolts, motorAmps,
                forwardLimitSwitch,
                reverseLimitSwitch);

        super.meters = motorRotsToMechMeters(motorRots.getValue());
        super.metersPerSecond = motorRotsToMechMeters(motorVelo.getValue());
        super.volts = motorVolts.getValue();
        super.amps = motorAmps.getValue();
        super.isLimitFwdSwitchHit = forwardLimitSwitch.getValue() == ForwardLimitValue.Open;
        super.isLimitRevSwitchHit = reverseLimitSwitch.getValue() == ReverseLimitValue.Open;

        if (!hasHomed && (super.isLimitFwdSwitchHit || super.isLimitRevSwitchHit)) {
            hasHomed = true;
        }

        if (hasHomed && motorAutoseed) {
            HardwareLimitSwitchConfigs cfg = new HardwareLimitSwitchConfigs();
            var configurator = motor.getConfigurator();
            configurator.refresh(cfg);
            cfg.ForwardLimitAutosetPositionEnable = false;
            cfg.ReverseLimitAutosetPositionEnable = false;
            configurator.apply(cfg);
            motorAutoseed = false;
        }
    }

}
