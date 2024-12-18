package com.igknighters.subsystems.stem.telescope;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.FaultManager;

import monologue.Annotations.Log;

/**
 * A {@link RealTelescope} variant that doesn't use the forward limit switch.
 * 
 * The forward limit switch is an IR beam break sensor that is used to detect when the telescope is fully extended.
 * This is unreliable in the sun and thus has to be disabled when using the robot outside.
 */
public class TelescopeRealSunshine extends Telescope {
    private final TalonFX motor;

    private final StatusSignal<Double> motorVolts, motorAmps, motorVelo, motorRots;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final MotionMagicTorqueCurrentFOC controlReqMotionMagic = new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    @Log
    private boolean hasHomed = false;
    @Log
    private boolean motorAutoseed = true;

    public TelescopeRealSunshine() {
        super(kTelescope.MIN_METERS);

        motor = new TalonFX(kTelescope.MOTOR_ID, kStem.CANBUS);
        CANRetrier.retryStatusCodeFatal(() -> motor.getConfigurator().apply(motorConfig(), 1.0), 10);

        motorRots = motor.getRotorPosition();
        motorVelo = motor.getRotorVelocity();
        motorAmps = motor.getTorqueCurrent();
        motorVolts = motor.getMotorVoltage();

        reverseLimitSwitch = motor.getReverseLimit();

        CANSignalManager.registerSignals(
                kStem.CANBUS,
                motorRots, motorVelo, motorVolts, motorAmps,
                reverseLimitSwitch);

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

        cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

        cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
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
        this.motor.setControl(
            controlReqVolts.withOutput(volts)
        );
    }

    @Override
    public void gotoPosition(double meters) {
        super.targetMeters = meters;
        this.motor.setControl(
            controlReqMotionMagic.withPosition(mechMetersToMotorRots(meters))
        );
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
                reverseLimitSwitch);

        super.meters = motorRotsToMechMeters(motorRots.getValueAsDouble());
        super.metersPerSecond = motorRotsToMechMeters(motorVelo.getValueAsDouble());
        super.volts = motorVolts.getValueAsDouble();
        super.amps = motorAmps.getValueAsDouble();
        super.isLimitFwdSwitchHit = false;
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
