package com.igknighters.subsystems.stem.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.util.SafeTalonFXConfiguration;

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

    private final PivotInputs inputs;

    private double mechRadiansToMotorRots(Double mechRads) {
        return (mechRads / (2 * Math.PI)) * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private double motorRotsToMechRadians(Double motorRots) {
        return (motorRots * (2 * Math.PI)) / kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    public PivotReal() {
        gyro = new Pigeon2(kPivot.PIGEON_ID);
        gyroMeasurement = gyro.getRoll();

        gyroMeasurement.setUpdateFrequency(100);

        gyro.optimizeBusUtilization();

        leaderMotor = new TalonFX(kPivot.LEFT_MOTOR_ID);
        followerMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID);

        FaultManager.captureFault(
                StemHW.LeaderMotor,
                leaderMotor.getConfigurator().apply(getMotorConfig()));
        FaultManager.captureFault(
                StemHW.FollowerMotor,
                followerMotor.getConfigurator().apply(getMotorConfig()));

        FaultManager.captureFault(
                StemHW.FollowerMotor,
                followerMotor.setControl(
                        new Follower(kPivot.LEFT_MOTOR_ID, true)));

        inputs = new PivotInputs(Units.degreesToRadians(gyroMeasurement.getValue()) - kPivot.PIGEON_OFFSET);

        leaderMotor.setPosition(mechRadiansToMotorRots(getPivotRadiansPigeon()));

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        followerMotorAmps = leaderMotor.getTorqueCurrent();
        leaderMotorAmps = followerMotor.getTorqueCurrent();
        leaderMotorVolts = leaderMotor.getMotorVoltage();
        followerMotorVolts = followerMotor.getMotorVoltage();
        followerMotorTemp = leaderMotor.getDeviceTemp();
        leaderMotorTemp = followerMotor.getDeviceTemp();

        motorRots.setUpdateFrequency(100);
        motorVelo.setUpdateFrequency(100);
        followerMotorAmps.setUpdateFrequency(100);
        leaderMotorAmps.setUpdateFrequency(100);
        leaderMotorVolts.setUpdateFrequency(100);
        followerMotorVolts.setUpdateFrequency(100);
        followerMotorTemp.setUpdateFrequency(4);
        leaderMotorTemp.setUpdateFrequency(4);

        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();

        followerMotor.setControl(
                new Follower(kPivot.LEFT_MOTOR_ID, true));

        BootupLogger.bootupLog("    Pivot initialized (real)");
    }

    private TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration motorCfg = new SafeTalonFXConfiguration();
        motorCfg.Slot0.kP = kPivot.MOTOR_kP;
        motorCfg.Slot0.kI = kPivot.MOTOR_kI;
        motorCfg.Slot0.kD = kPivot.MOTOR_kD;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorCfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        motorCfg.HardwareLimitSwitch.ReverseLimitEnable = true;
        motorCfg.HardwareLimitSwitch.ForwardLimitEnable = true;

        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mechRadiansToMotorRots(kPivot.PIVOT_MAX_RADIANS);
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mechRadiansToMotorRots(kPivot.PIVOT_MIN_RADIANS);

        motorCfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        motorCfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        return motorCfg;
    }

    @Override
    public void setPivotRadians(double radians) {
        if (radians > kPivot.PIVOT_MAX_RADIANS || radians < kPivot.PIVOT_MIN_RADIANS) {
            String errorMsg = "Pivot setpoint of " + radians + " radians is outside the scope of minimum "
                    + kPivot.PIVOT_MIN_RADIANS + " radians and maximum " + kPivot.PIVOT_MAX_RADIANS + " radians!";
            DriverStation.reportWarning(errorMsg, false);
            return;
        }
        inputs.targetRadians = radians;
        this.leaderMotor.setControl(new MotionMagicDutyCycle(mechRadiansToMotorRots(radians)));
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
        return inputs.gyroRadians - kPivot.PIGEON_OFFSET;
    }

    private void seedPivot() {
        leaderMotor.setPosition(mechRadiansToMotorRots(getPivotRadiansPigeon()));
        inputs.radians = getPivotRadiansPigeon();
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                StemHW.LeaderMotor,
                BaseStatusSignal.refreshAll(
                        motorRots, motorVelo,
                        leaderMotorVolts, leaderMotorAmps,
                        leaderMotorTemp));

        FaultManager.captureFault(
                StemHW.FollowerMotor,
                BaseStatusSignal.refreshAll(
                        followerMotorAmps, followerMotorTemp, followerMotorVolts));

        FaultManager.captureFault(
                StemHW.Pigeon2,
                gyroMeasurement.refresh().getStatus());

        inputs.radians = motorRotsToMechRadians(motorRots.getValue());
        inputs.radiansPerSecond = motorRotsToMechRadians(motorVelo.getValue());
        inputs.gyroRadians = Math.abs(Units.degreesToRadians(gyroMeasurement.getValue()));
        inputs.leftVolts = leaderMotorVolts.getValue();
        inputs.rightVolts = followerMotorVolts.getValue();
        inputs.leftAmps = leaderMotorAmps.getValue();
        inputs.rightAmps = followerMotorAmps.getValue();
        inputs.leftTemp = leaderMotorTemp.getValue();
        inputs.rightTemp = followerMotorTemp.getValue();

        if (Math.abs(inputs.radiansPerSecond) < 0.33
                && inputs.radians < Math.PI / 2.2 // ensuers backlash is going the right way (kinda)
        ) {
            seedPivot();
            Logger.recordOutput("Stem/Pivot/SeededPivot", true);
        } else {
            Logger.recordOutput("Stem/Pivot/SeededPivot", false);
        }

        Logger.processInputs("Stem/Pivot", inputs);
    }

}
