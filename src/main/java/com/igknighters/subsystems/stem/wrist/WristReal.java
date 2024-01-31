package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.SafeTalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristReal implements Wrist {
    private final TalonFX motor;
    private final CANcoder cancoder;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts, motorTemp;
    private final StatusSignal<Double> cancoderRots, cancoderVelo;

    private final WristInputs inputs;

    public WristReal() {
        motor = new TalonFX(kWrist.MOTOR_ID);
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

        cancoder = new CANcoder(kWrist.CANCODER_ID);
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

    private double mechanismRadsToMotorRots(Double radians) {
        double clampedRadians = MathUtil.clamp(
            radians,
            kWrist.WRIST_MIN_ANGLE,
            kWrist.WRIST_MAX_ANGLE
        );

        if (clampedRadians != radians) {
            DriverStation.reportWarning(
                "[Wrist] requested mechanism rads out of bounds: " + radians
                + " (clamped to " + clampedRadians + ")",
                false);
        }

        SmartDashboard.putNumber("clampedrads", clampedRadians);

        double scopedRadians = (Math.PI / 2.0) - (clampedRadians - (Math.PI / 2.0));

        double beta = scopedRadians - kWrist.kDimensions.ANGLE_OFFSET;

        SmartDashboard.putNumber("beta", beta);

        double leadscrewLength = Math.sqrt(
            Math.pow(kWrist.kDimensions.MOTOR_PIVOT_TO_WRIST_PIVOT, 2)
            + Math.pow(kWrist.kDimensions.WRIST_PIVOT_TO_NUT, 2)
            - (2 * kWrist.kDimensions.MOTOR_PIVOT_TO_WRIST_PIVOT * kWrist.kDimensions.WRIST_PIVOT_TO_NUT * Math.cos(beta))
        );

        SmartDashboard.putNumber("leadscrewInches", Units.metersToInches(leadscrewLength));

        return Units.metersToInches(leadscrewLength) * 20.0;
    }

    @Override
    public void setWristRadians(Double radians) {
        inputs.targetRadians = radians;
        var posControlRequest = new PositionDutyCycle(
                mechanismRadsToMotorRots(radians));
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
    public void periodic() {
        BaseStatusSignal.refreshAll(
                motorAmps, motorVolts,
                motorRots, motorVelo,
                motorTemp, cancoderRots,
                cancoderVelo);

        motor.setPosition(
            mechanismRadsToMotorRots(
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
