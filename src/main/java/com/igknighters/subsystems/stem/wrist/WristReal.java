package com.igknighters.subsystems.stem.wrist;

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
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;

import edu.wpi.first.math.util.Units;

public class WristReal extends Wrist {
    private final TalonFX motor;
    private final CANcoder cancoder;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts;
    private final StatusSignal<Double> cancoderRots, cancoderVelo;

    public WristReal() {
        super(0.0);
        motor = new TalonFX(kWrist.MOTOR_ID, kStem.CANBUS);
        motor.getConfigurator().apply(motorConfig());

        motorRots = motor.getRotorPosition();
        motorVelo = motor.getRotorVelocity();
        motorAmps = motor.getTorqueCurrent();
        motorVolts = motor.getMotorVoltage();

        cancoder = new CANcoder(kWrist.CANCODER_ID, kStem.CANBUS);
        cancoder.getConfigurator().apply(cancoderConfig());

        cancoderRots = cancoder.getAbsolutePosition();
        cancoderVelo = cancoder.getVelocity();

        super.encoderRadians = Units.rotationsToRadians(cancoderRots.getValue());
        super.radians = encoderRadians;
        super.targetRadians = encoderRadians;
        seedWrist();

        CANSignalManager.registerSignals(
            kStem.CANBUS,
            motorRots, motorVelo, motorAmps, motorVolts,
            cancoderRots, cancoderVelo
        );

        motor.optimizeBusUtilization(1.0);
        cancoder.optimizeBusUtilization(1.0);

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

        return cfg;
    }

    private CANcoderConfiguration cancoderConfig() {
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cfg.MagnetSensor.MagnetOffset = kWrist.CANCODER_OFFSET;

        return cfg;
    }

    public void seedWrist() {
        CANRetrier.retryStatusCodeFatal(() -> motor.setPosition(Wrist.mechanismRadsToMotorRots(super.encoderRadians)),
                10);
    }

    public double getAmps() {
        return super.amps;
    }

    @Override
    public void setWristRadians(double radians) {
        super.targetRadians = radians;
        var posControlRequest = new PositionTorqueCurrentFOC(
                Wrist.mechanismRadsToMotorRots(radians));
        this.motor.setControl(posControlRequest);
    }

    @Override
    public double getWristRadians() {
        return super.radians;
    }

    @Override
    public void setVoltageOut(double volts) {
        super.targetRadians = 0.0;
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
                    motorRots, motorVelo,
                    motorVolts, motorAmps);

        FaultManager.captureFault(
                StemHW.WristEncoder,
                    cancoderRots,
                    cancoderVelo);

        super.radians = Wrist.motorRotsToMechanismRads(motorRots.getValue());
        super.radiansPerSecond = Units.rotationsToRadians(cancoderVelo.getValue());
        super.encoderRadians = Units.rotationsToRadians(cancoderRots.getValue());
        super.amps = motorAmps.getValue();
        super.volts = motorVolts.getValue();

        if (Math.abs(super.radiansPerSecond) < 0.1
                && Math.abs(super.radians - super.encoderRadians) > 0.1) {
            seedWrist();
            log("SeededWrist", true);
        } else {
            log("SeededWrist", false);
        }
    }

}
