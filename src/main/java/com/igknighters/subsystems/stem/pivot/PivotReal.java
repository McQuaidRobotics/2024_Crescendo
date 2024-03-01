package com.igknighters.subsystems.stem.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.CANRetrier;
import com.igknighters.util.FaultManager;

public class PivotReal implements Pivot {

    /** Left */
    private final TalonFX leaderMotor;
    /** Right */
    private final TalonFX followerMotor;

    private final Pigeon2 gyro;

    private final StatusSignal<Double> motorRots, motorVelo, leaderMotorVolts, followerMotorVolts;
    private final StatusSignal<Double> leaderMotorAmps, followerMotorAmps, leaderMotorTemp, followerMotorTemp;
    /** Could be pitch or roll */
    private final StatusSignal<Double> gyroMeasurement;
    private final StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final PivotInputs inputs;

    private double mechRadiansToMotorRots(Double mechRads) {
        return Units.radiansToRotations(Math.PI - mechRads) * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private double motorRotsToMechRadians(Double motorRots) {
        return Math.PI - Units.rotationsToRadians(motorRots / kPivot.MOTOR_TO_MECHANISM_RATIO);
    }

    public PivotReal() {
        gyro = new Pigeon2(kPivot.PIGEON_ID, kStem.CANBUS);
        gyroMeasurement = gyro.getPitch();

        gyroMeasurement.setUpdateFrequency(100);

        gyro.optimizeBusUtilization();

        leaderMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID, kStem.CANBUS);
        followerMotor = new TalonFX(kPivot.LEFT_MOTOR_ID, kStem.CANBUS);

        CANRetrier.retryStatusCodeFatal(() -> leaderMotor.getConfigurator().apply(getMotorConfig(true)), 10);
        CANRetrier.retryStatusCodeFatal(() -> followerMotor.getConfigurator().apply(getMotorConfig(false)), 10);
        CANRetrier.retryStatusCodeFatal(() -> followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true)), 10);

        // FaultManager.captureFault(
        //     StemHW.LeaderMotor,
        //     leaderMotor.getConfigurator().apply(getMotorConfig(true)));
        //     FaultManager.captureFault(
        //         StemHW.FollowerMotor,
        //         followerMotor.getConfigurator().apply(getMotorConfig(false)));
                
        // FaultManager.captureFault(
        //     StemHW.FollowerMotor,
        //     followerMotor.setControl(
        //         new Follower(leaderMotor.getDeviceID(), true)));

        inputs = new PivotInputs(Units.degreesToRadians(gyroMeasurement.getValue()));

        leaderMotor.setPosition(mechRadiansToMotorRots(getPivotRadiansPigeon()));

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        followerMotorAmps = leaderMotor.getTorqueCurrent();
        leaderMotorAmps = followerMotor.getTorqueCurrent();
        leaderMotorVolts = leaderMotor.getMotorVoltage();
        followerMotorVolts = followerMotor.getMotorVoltage();
        followerMotorTemp = leaderMotor.getDeviceTemp();
        leaderMotorTemp = followerMotor.getDeviceTemp();
        forwardLimitSwitch = leaderMotor.getForwardLimit();
        reverseLimitSwitch = leaderMotor.getReverseLimit();

        motorRots.setUpdateFrequency(100);
        motorVelo.setUpdateFrequency(100);
        followerMotorAmps.setUpdateFrequency(100);
        leaderMotorAmps.setUpdateFrequency(100);
        leaderMotorVolts.setUpdateFrequency(100);
        followerMotorVolts.setUpdateFrequency(100);
        followerMotorTemp.setUpdateFrequency(4);
        leaderMotorTemp.setUpdateFrequency(4);
        forwardLimitSwitch.setUpdateFrequency(100);
        reverseLimitSwitch.setUpdateFrequency(100);

        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();

        seedPivot();

        BootupLogger.bootupLog("    Pivot initialized (real)");
    }

    private TalonFXConfiguration getMotorConfig(boolean leader) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kPivot.MOTOR_kP;
        cfg.Slot0.kI = kPivot.MOTOR_kI;
        cfg.Slot0.kD = kPivot.MOTOR_kD;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        cfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        // mechRadiansToMotorRots(kPivot.PIVOT_MAX_RADIANS);
        // cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        // mechRadiansToMotorRots(kPivot.PIVOT_MIN_RADIANS);

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

        // if (leader) {
        //     cfg.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
        //     cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;
        //     cfg.HardwareLimitSwitch.ForwardLimitRemoteSensorID = kPivot.RIGHT_MOTOR_ID;
        //     cfg.HardwareLimitSwitch.ReverseLimitRemoteSensorID = kPivot.RIGHT_MOTOR_ID;
        // }

        return cfg;
    }

    @Override
    public void setPivotRadians(double radians) {
        inputs.targetRadians = radians;
        this.leaderMotor.setControl(new MotionMagicVoltage(mechRadiansToMotorRots(radians)));
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.targetRadians = 0.0;
        this.leaderMotor.setVoltage(volts);
    }

    @Override
    public double getPivotRadians() {
        return inputs.radians;
    }

    private double getPivotRadiansPigeon() {
        return inputs.gyroRadians;
    }

    private void seedPivot() {
        leaderMotor.setPosition(mechRadiansToMotorRots(getPivotRadiansPigeon()));
        inputs.radians = getPivotRadiansPigeon();
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
                BaseStatusSignal.refreshAll(
                        motorRots /*, motorVelo, */
                        /* leaderMotorVolts, leaderMotorAmps, */
                        /* leaderMotorTemp,  reverseLimitSwitch, */
                        /* forwardLimitSwitch */));

        // FaultManager.captureFault(
        //         StemHW.FollowerMotor,
        //         BaseStatusSignal.refreshAll(
        //                 followerMotorAmps, followerMotorTemp, followerMotorVolts));

        FaultManager.captureFault(
                StemHW.Pigeon2,
                gyroMeasurement.refresh().getStatus());

        inputs.radians = motorRotsToMechRadians(motorRots.getValue());
        inputs.radiansPerSecond = -Units.rotationsToRadians(motorVelo.getValue()) / kPivot.MOTOR_TO_MECHANISM_RATIO;
        inputs.leftVolts = leaderMotorVolts.getValue();
        inputs.rightVolts = followerMotorVolts.getValue();
        inputs.leftAmps = leaderMotorAmps.getValue();
        inputs.rightAmps = followerMotorAmps.getValue();
        inputs.leftTemp = leaderMotorTemp.getValue();
        inputs.rightTemp = followerMotorTemp.getValue();

        inputs.isLimitFwdSwitchHit = forwardLimitSwitch.getValue() == ForwardLimitValue.Open;
        inputs.isLimitRevSwitchHit = reverseLimitSwitch.getValue() == ReverseLimitValue.Open;

        inputs.gyroRadians = Units.degreesToRadians(gyroMeasurement.getValue() + 90);

        if (Math.abs(inputs.radiansPerSecond) < 0.1
                && Math.abs(inputs.radians - getPivotRadiansPigeon()) > 0.1 && !GlobalState.isClimbing()) {
            seedPivot();
            Logger.recordOutput("Stem/Pivot/SeededPivot", true);
        } else {
            Logger.recordOutput("Stem/Pivot/SeededPivot", false);
        }

        Logger.recordOutput("Stem/Pivot/GyroSeed", mechRadiansToMotorRots(getPivotRadiansPigeon()));

        Logger.processInputs("Stem/Pivot", inputs);
    }

}
