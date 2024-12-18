package com.igknighters.subsystems.stem.pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.util.logging.FaultManager;

public class PivotReal extends Pivot {

    /** Right */
    private final TalonFX leaderMotor;
    /** Left */
    private final TalonFX followerMotor;

    private final Pigeon2 gyro;

    private final StatusSignal<Double> motorRots, motorVelo, leaderMotorVolts, followerMotorVolts;
    private final StatusSignal<Double> leaderMotorAmps, followerMotorAmps;
    private final StatusSignal<Double> gyroMeasurement;
    private final StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final MotionMagicVoltage controlReqMotionMagic = new MotionMagicVoltage(0.0).withUpdateFreqHz(0);

    private boolean homedThisCycle = false;
    private boolean hasBeenEnabled = false;

    private double mechRadiansToMotorRots(Double mechRads) {
        return Units.radiansToRotations(Math.PI - mechRads) * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private double motorRotsToMechRadians(Double motorRots) {
        return Math.PI - Units.rotationsToRadians(motorRots / kPivot.MOTOR_TO_MECHANISM_RATIO);
    }

    public PivotReal() {
        super(0.0);
        gyro = new Pigeon2(kPivot.PIGEON_ID, kStem.CANBUS);
        gyroMeasurement = gyro.getPitch();

        leaderMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID, kStem.CANBUS);
        followerMotor = new TalonFX(kPivot.LEFT_MOTOR_ID, kStem.CANBUS);

        CANRetrier.retryStatusCodeFatal(() -> leaderMotor.getConfigurator().apply(getMotorConfig(true)), 10);
        CANRetrier.retryStatusCodeFatal(() -> followerMotor.getConfigurator().apply(getMotorConfig(false)), 10);
        CANRetrier.retryStatusCodeFatal(() -> followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true)),
                10);

        double startingRads = Units.degreesToRadians(gyroMeasurement.getValueAsDouble());
        super.gyroRadians = startingRads;
        super.radians = startingRads;
        super.targetRadians = startingRads;

        home();

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        followerMotorAmps = leaderMotor.getTorqueCurrent();
        leaderMotorAmps = followerMotor.getTorqueCurrent();
        leaderMotorVolts = leaderMotor.getMotorVoltage();
        followerMotorVolts = followerMotor.getMotorVoltage();
        forwardLimitSwitch = leaderMotor.getForwardLimit();
        reverseLimitSwitch = leaderMotor.getReverseLimit();

        CANSignalManager.registerSignals(
            kStem.CANBUS,
            motorRots, motorVelo, leaderMotorVolts, followerMotorVolts,
            leaderMotorAmps, followerMotorAmps, forwardLimitSwitch,
            reverseLimitSwitch, gyroMeasurement
        );

        gyro.optimizeBusUtilization(1.0);
        leaderMotor.optimizeBusUtilization(1.0);
        followerMotor.optimizeBusUtilization(1.0);

        BootupLogger.bootupLog("    Pivot initialized (real)");
    }

    private TalonFXConfiguration getMotorConfig(boolean leader) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kPivot.MOTOR_kP;
        cfg.Slot0.kI = kPivot.MOTOR_kI;
        cfg.Slot0.kD = kPivot.MOTOR_kD;
        cfg.Slot0.kS = kPivot.MOTOR_kS;
        cfg.Slot0.kV = kPivot.MOTOR_kV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        cfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        cfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        cfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        if (leader) {
            cfg.HardwareLimitSwitch.ForwardLimitEnable = true;
            cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
        } else {
            cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
            cfg.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        cfg.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
        cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

        return cfg;
    }

    @Override
    public void gotoPosition(double radians) {
        super.targetRadians = radians;
        this.leaderMotor.setControl(controlReqMotionMagic.withPosition(mechRadiansToMotorRots(radians)));
    }

    @Override
    public void setVoltageOut(double volts) {
        super.targetRadians = 0.0;
        this.leaderMotor.setControl(controlReqVolts.withOutput(volts));
    }

    private double getPivotRadiansPigeon() {
        return super.gyroRadians;
    }

    @Override
    public void home() {
        leaderMotor.setPosition(mechRadiansToMotorRots(getPivotRadiansPigeon()), 0.01);
        super.radians = getPivotRadiansPigeon();
        homedThisCycle = true;
    }

    @Override
    public void setCoast(boolean shouldBeCoasting) {
        this.followerMotor.setNeutralMode(
                shouldBeCoasting
                        ? NeutralModeValue.Coast
                        : NeutralModeValue.Brake);
        this.leaderMotor.setNeutralMode(
                shouldBeCoasting
                        ? NeutralModeValue.Coast
                        : NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
            StemHW.LeaderMotor,
            motorRots, motorVelo,
            leaderMotorVolts, leaderMotorAmps,
            forwardLimitSwitch, reverseLimitSwitch
        );

        FaultManager.captureFault(
            StemHW.FollowerMotor,
            followerMotorAmps, followerMotorVolts
        );

        FaultManager.captureFault(
            StemHW.Pigeon2,
            gyroMeasurement
        );

        super.radians = motorRotsToMechRadians(motorRots.getValueAsDouble());
        super.radiansPerSecond = -Units.rotationsToRadians(motorVelo.getValueAsDouble()) / kPivot.MOTOR_TO_MECHANISM_RATIO;
        super.leftVolts = leaderMotorVolts.getValueAsDouble();
        super.rightVolts = followerMotorVolts.getValueAsDouble();
        super.leftAmps = leaderMotorAmps.getValueAsDouble();
        super.rightAmps = followerMotorAmps.getValueAsDouble();

        super.isLimitFwdSwitchHit = forwardLimitSwitch.getValue() == ForwardLimitValue.Open;
        super.isLimitRevSwitchHit = reverseLimitSwitch.getValue() == ReverseLimitValue.Open;

        double newGyroRadians = Units.degreesToRadians(gyroMeasurement.getValueAsDouble() + 90.0);
        super.gyroRadiansPerSecondAbs = Math.abs(super.gyroRadians - newGyroRadians) / ConstValues.PERIODIC_TIME;
        super.gyroRadians = newGyroRadians;

        if (DriverStation.isEnabled()) {
            hasBeenEnabled = true;
        }

        if (Math.abs(super.radiansPerSecond) < 0.01
                && Math.abs(super.gyroRadiansPerSecondAbs) < 0.01
                && (Units.radiansToDegrees(super.gyroRadians) > 20.0 || !hasBeenEnabled)
                && DriverStation.isDisabled()
        ) {
            home();
        }

        log("SeededPivot", homedThisCycle);
        homedThisCycle = false;
    }

}
