package com.igknighters.subsystems.stem.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.util.logging.FaultManager;

import edu.wpi.first.math.util.Units;

public class WristRealFused extends Wrist {
    private final TalonFX motor;
    private final CANcoder cancoder;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts;
    private final StatusSignal<Double> cancoderRots, cancoderVelo;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final MotionMagicVoltage controlReqMotionMagic = new MotionMagicVoltage(0.0).withUpdateFreqHz(0)
        .withEnableFOC(true);

    public WristRealFused() {
        super(0.0);
        motor = new TalonFX(kWrist.MOTOR_ID, kStem.CANBUS);
        CANRetrier.retryStatusCodeFatal(() -> motor.getConfigurator().apply(motorConfig()), 10);

        motorRots = motor.getPosition();
        motorVelo = motor.getVelocity();
        motorAmps = motor.getTorqueCurrent();
        motorVolts = motor.getMotorVoltage();

        cancoder = new CANcoder(kWrist.CANCODER_ID, kStem.CANBUS);
        CANRetrier.retryStatusCodeFatal(() -> cancoder.getConfigurator().apply(cancoderConfig()), 10);

        cancoderRots = cancoder.getAbsolutePosition();
        cancoderVelo = cancoder.getVelocity();

        CANSignalManager.registerSignals(
            kStem.CANBUS,
            motorVelo, motorRots,
            motorAmps, motorVolts,
            cancoderRots, cancoderVelo
        );

        cancoder.optimizeBusUtilization(1.0);
        motor.optimizeBusUtilization(1.0);

        super.encoderRadians = Units.rotationsToRadians(cancoderRots.getValueAsDouble());
        super.radians = encoderRadians;
        super.targetRadians = encoderRadians;

        BootupLogger.bootupLog("    Wrist initialized (real)");
    }

    private TalonFXConfiguration motorConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kWrist.MOTOR_kP;
        cfg.Slot0.kI = kWrist.MOTOR_kI;
        cfg.Slot0.kD = kWrist.MOTOR_kD;
        cfg.Slot0.kS = kWrist.MOTOR_kS;
        cfg.Slot0.kV = kWrist.MOTOR_kV;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = kWrist.INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kWrist.MAX_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = kWrist.MAX_ACCELERATION;
        cfg.MotionMagic.MotionMagicJerk = kWrist.MAX_JERK;

        cfg.Feedback.FeedbackRemoteSensorID = kWrist.CANCODER_ID;
        cfg.Feedback.RotorToSensorRatio = kWrist.MOTOR_TO_MECHANISM_RATIO;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        return cfg;
    }

    private CANcoderConfiguration cancoderConfig() {
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cfg.MagnetSensor.MagnetOffset = kWrist.CANCODER_OFFSET;

        return cfg;
    }

    @Override
    public void gotoPosition(double targetRadians) {
        super.targetRadians = targetRadians;
        this.motor.setControl(
            controlReqMotionMagic.withPosition(Units.radiansToRotations(targetRadians))
        );
    }

    @Override
    public void setVoltageOut(double volts) {
        super.targetRadians = 0.0;
        motor.setControl(
            controlReqVolts.withOutput(volts)
        );
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
                cancoderRots);

        super.radians = Units.rotationsToRadians(motorRots.getValueAsDouble());
        super.radiansPerSecond = Units.rotationsToRadians(cancoderVelo.getValueAsDouble());
        super.encoderRadians = Units.rotationsToRadians(cancoderRots.getValueAsDouble());
        super.amps = motorAmps.getValueAsDouble();
        super.volts = motorVolts.getValueAsDouble();
    }
}
