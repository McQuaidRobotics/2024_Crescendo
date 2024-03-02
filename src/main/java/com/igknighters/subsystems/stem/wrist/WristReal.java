package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.CANRetrier;
import com.igknighters.util.FaultManager;

import edu.wpi.first.math.util.Units;

public class WristReal implements Wrist {
    private final TalonFX motor;
    private final CANcoder cancoder;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts, motorTemp;
    private final StatusSignal<Double> cancoderRots, cancoderVelo;

    private final WristInputs inputs;

    public WristReal() {
        motor = new TalonFX(kWrist.MOTOR_ID, kStem.CANBUS);
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

        motor.optimizeBusUtilization();

        cancoder = new CANcoder(kWrist.CANCODER_ID, kStem.CANBUS);
        cancoder.getConfigurator().apply(cancoderConfig());

        cancoderRots = cancoder.getAbsolutePosition();
        cancoderVelo = cancoder.getVelocity();

        cancoderRots.setUpdateFrequency(100);
        cancoderVelo.setUpdateFrequency(100);

        cancoder.optimizeBusUtilization();

        inputs = new WristInputs(Units.rotationsToRadians(cancoderRots.getValue()));
        seedWrist();

        BootupLogger.bootupLog("    Wrist initialized (real)");
    }

    private TalonFXConfiguration motorConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        cfg.Slot0.kP = kWrist.MOTOR_kP;
        cfg.Slot0.kI = kWrist.MOTOR_kI;
        cfg.Slot0.kD = kWrist.MOTOR_kD;
        cfg.Slot0.kS = kWrist.MOTOR_kS;
        cfg.Slot0.kV = kWrist.MOTOR_kV;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = kWrist.INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Wrist.mechanismRadsToMotorRots(StemPosition.STARTING.wristRads - 0.3);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Wrist.mechanismRadsToMotorRots(kWrist.FROZEN_WRIST_ANGLE - Units.degreesToRadians(0.3));

        return cfg;
    }

    private CANcoderConfiguration cancoderConfig() {
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cfg.MagnetSensor.MagnetOffset = kWrist.CANCODER_OFFSET;

        return cfg;
    }

    public void seedWrist() {
        CANRetrier.retryStatusCode(() ->motor.setPosition(Wrist.mechanismRadsToMotorRots(inputs.radians)), 10);
    }

    public double getAmps() {
        return inputs.amps;
    }

    @Override
    public void setWristRadians(double radians) {
        inputs.targetRadians = radians;
        var posControlRequest = new PositionTorqueCurrentFOC(
                Wrist.mechanismRadsToMotorRots(radians));
        this.motor.setControl(posControlRequest);
    }

    @Override
    public double getWristRadians() {
        return inputs.radians;
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.targetRadians = 0.0;
        motor.setVoltage(volts);
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
                StemHW.WristMotor,
                BaseStatusSignal.refreshAll(
                        motorRots, motorVelo,
                        motorVolts, motorAmps,
                        motorTemp));

        FaultManager.captureFault(
                StemHW.WristEncoder,
                BaseStatusSignal.refreshAll(
                        cancoderRots));

        inputs.radians = Wrist.motorRotsToMechanismRads(motorRots.getValue());
        inputs.radiansPerSecond = Units.rotationsToRadians(cancoderVelo.getValue());
        Logger.recordOutput("Stem/Wrist/EncoderRads", Units.rotationsToRadians(cancoderRots.getValue()));
        inputs.amps = motorAmps.getValue();
        inputs.volts = motorVolts.getValue();
        inputs.temp = motorTemp.getValue();

        // if (Math.abs(inputs.radiansPerSecond) < 0.1
        //         && Math.abs(inputs.radians - Wrist.motorRotsToMechanismRads(motorRots.getValue())) > 0.1) {
        //     seedWrist();
        //     Logger.recordOutput("Stem/Wrist/SeededWrist", true);
        // } else {
        //     Logger.recordOutput("Stem/Wrist/SeededWrist", false);
        // }

        Logger.processInputs("Stem/Wrist", inputs);
    }

}
