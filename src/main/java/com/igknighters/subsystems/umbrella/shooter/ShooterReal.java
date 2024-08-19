package com.igknighters.subsystems.umbrella.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.util.logging.FaultManager;
import com.igknighters.constants.HardwareIndex.UmbrellaHW;

import edu.wpi.first.math.util.Units;

public class ShooterReal extends Shooter {

    private final TalonFX rightMotor = new TalonFX(kShooter.RIGHT_MOTOR_ID, kUmbrella.CANBUS);
    private final TalonFX leftMotor = new TalonFX(kShooter.LEFT_MOTOR_ID, kUmbrella.CANBUS);
    private final StatusSignal<Double> veloSignalRight, voltSignalRight, currentSignalRight;
    private final StatusSignal<Double> veloSignalLeft, voltSignalLeft, currentSignalLeft;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final VelocityVoltage controlReq = new VelocityVoltage(0.0).withUpdateFreqHz(0).withEnableFOC(true);

    public ShooterReal() {
        rightMotor.getConfigurator().apply(motorRightConfig(), 1.0);
        leftMotor.getConfigurator().apply(motorLeftConfig(), 1.0);

        veloSignalRight = rightMotor.getVelocity();
        voltSignalRight = rightMotor.getMotorVoltage();
        currentSignalRight = rightMotor.getTorqueCurrent();

        veloSignalLeft = leftMotor.getVelocity();
        voltSignalLeft = leftMotor.getMotorVoltage();
        currentSignalLeft = leftMotor.getTorqueCurrent();

        CANSignalManager.registerSignals(
                kUmbrella.CANBUS,
                veloSignalRight, voltSignalRight, currentSignalRight,
                veloSignalLeft, voltSignalLeft, currentSignalLeft);

        rightMotor.optimizeBusUtilization(1.0);
        leftMotor.optimizeBusUtilization(1.0);

        BootupLogger.bootupLog("    Shooter initialized (real)");
    }

    private TalonFXConfiguration motorRightConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_RIGHT_kP;
        cfg.Slot0.kI = kShooter.MOTOR_RIGHT_kI;
        cfg.Slot0.kD = kShooter.MOTOR_RIGHT_kD;
        cfg.Slot0.kS = kShooter.MOTOR_RIGHT_kS;
        cfg.Slot0.kV = kShooter.MOTOR_RIGHT_kV;

        cfg.CurrentLimits.StatorCurrentLimit = kShooter.PEAK_CURRENT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;
        cfg.Voltage.PeakReverseVoltage = 0.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return cfg;
    }

    private TalonFXConfiguration motorLeftConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooter.MOTOR_LEFT_kP;
        cfg.Slot0.kI = kShooter.MOTOR_LEFT_kI;
        cfg.Slot0.kD = kShooter.MOTOR_LEFT_kD;
        cfg.Slot0.kS = kShooter.MOTOR_LEFT_kS;
        cfg.Slot0.kV = kShooter.MOTOR_LEFT_kV;

        cfg.CurrentLimits.StatorCurrentLimit = kShooter.PEAK_CURRENT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;
        cfg.Voltage.PeakReverseVoltage = 0.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return cfg;
    }

    @Override
    public double getSpeed() {
        return (super.radiansPerSecondRight + (super.radiansPerSecondLeft / kShooter.LEFT_MOTOR_DIFF)) / 2.0;
    }

    @Override
    public double getTargetSpeed() {
        return (super.targetRadiansPerSecondRight + (super.targetRadiansPerSecondLeft / kShooter.LEFT_MOTOR_DIFF)) / 2.0;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        super.targetRadiansPerSecondRight = speedRadPerSec;
        super.targetRadiansPerSecondLeft = speedRadPerSec * kShooter.LEFT_MOTOR_DIFF;
        double targetRotsRight = Units.radiansToRotations(super.targetRadiansPerSecondRight) / kShooter.MECHANISM_RATIO;
        double targetRotsLeft = Units.radiansToRotations(super.targetRadiansPerSecondLeft) / kShooter.MECHANISM_RATIO;
        log("targetRotsRight", targetRotsRight);
        log("targetRotsLeft", targetRotsLeft);
        rightMotor.setControl(controlReq.withVelocity(targetRotsRight));
        leftMotor.setControl(controlReq.withVelocity(targetRotsLeft));
    }

    @Override
    public void setVoltageOut(double volts) {
        super.targetRadiansPerSecondRight = 0.0;
        super.targetRadiansPerSecondLeft = 0.0;
        super.voltsRight = volts;
        super.voltsLeft = volts;
        rightMotor.setControl(controlReqVolts.withOutput(volts));
        leftMotor.setControl(controlReqVolts.withOutput(volts));
    }

    @Override
    public void periodic() {
        FaultManager.captureFault(
                UmbrellaHW.RightShooterMotor,
                veloSignalRight,
                voltSignalRight,
                currentSignalRight);

        FaultManager.captureFault(
                UmbrellaHW.LeftShooterMotor,
                veloSignalLeft,
                voltSignalLeft,
                currentSignalLeft);

        super.radiansPerSecondRight = Units.rotationsToRadians(veloSignalRight.getValueAsDouble() * kShooter.MECHANISM_RATIO);
        log("currentRotsRight", veloSignalRight.getValueAsDouble());
        super.voltsRight = voltSignalRight.getValueAsDouble();
        super.ampsRight = currentSignalRight.getValueAsDouble();

        super.radiansPerSecondLeft = Units.rotationsToRadians(veloSignalLeft.getValueAsDouble() * kShooter.MECHANISM_RATIO);
        log("currentRotsLeft", veloSignalLeft.getValueAsDouble());
        super.voltsLeft = voltSignalLeft.getValueAsDouble();
        super.ampsLeft = currentSignalLeft.getValueAsDouble();

        super.shooterRightRPM = Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondRight);
        super.shooterLeftRPM = Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondLeft);
    }
}
