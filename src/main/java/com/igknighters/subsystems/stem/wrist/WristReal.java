package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.util.SafeTalonFXConfiguration;

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

        BootupLogger.bootupLog("    Wrist initialized (real)");
    }

    private TalonFXConfiguration motorConfig() {
        TalonFXConfiguration wristMotorCfg = new SafeTalonFXConfiguration();
        wristMotorCfg.Slot0.kP = kWrist.MOTOR_kP;
        wristMotorCfg.Slot0.kI = kWrist.MOTOR_kI;
        wristMotorCfg.Slot0.kD = kWrist.MOTOR_kD;

        wristMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristMotorCfg.MotorOutput.Inverted = kWrist.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        return wristMotorCfg;
    }

    private CANcoderConfiguration cancoderConfig() {
        CANcoderConfiguration wristCancoderCfg = new CANcoderConfiguration();
        wristCancoderCfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        wristCancoderCfg.MagnetSensor.MagnetOffset = kWrist.CANCODER_OFFSET;

        return wristCancoderCfg;
    }

    @Override
    public void setWristRadians(Double radians) {
        inputs.targetRadians = radians;
        var posControlRequest = new PositionVoltage(
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
            : NeutralModeValue.Brake
        );
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
                        cancoderRots, cancoderVelo));

        motor.setPosition(
            Wrist.mechanismRadsToMotorRots(
                Units.rotationsToRadians(cancoderRots.getValue())
        ));

        inputs.radians = Units.rotationsToRadians(cancoderRots.getValue());
        inputs.radiansPerSecond = Units.rotationsToRadians(cancoderVelo.getValue());
        Logger.recordOutput("Stem/Wrist/MotorRads", Units.rotationsToRadians(motorRots.getValue()));
        Logger.recordOutput("Stem/Wrist/MotorRadsPs", Units.rotationsToRadians(motorVelo.getValue()));
        inputs.amps = motorAmps.getValue();
        inputs.volts = motorVolts.getValue();
        inputs.temp = motorTemp.getValue();

        Logger.processInputs("Stem/Wrist", inputs);
    }

}
