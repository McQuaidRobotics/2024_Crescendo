package com.igknighters.subsystems.rookie;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kMotors;
import com.igknighters.subsystems.SubsystemResources.LockFullSubsystem;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.math.util.Units;

public class Launcher implements LockFullSubsystem {
    private static final int MOTOR_ID = 11;
    private static final String CANBUS = "rio";
    private static final double PEAK_CURRENT = 60.0;
    private static final double MECHANISM_RATIO = 1.0;

    private final TalonFX motor = new TalonFX(MOTOR_ID, CANBUS);
    private final StatusSignal<Double> veloSignal, voltSignal, currentSignal;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final VelocityVoltage controlReq = new VelocityVoltage(0.0).withUpdateFreqHz(0).withEnableFOC(true);

    private double radiansPerSecond, targetRadiansPerSecond;

    public Launcher() {
        if (Robot.isSimulation()) {
            throw new UnsupportedOperationException("Launcher subsystem is not supported in simulation");
        }

        motor.getConfigurator().apply(motorConfig(), 1.0);

        veloSignal = motor.getVelocity();
        voltSignal = motor.getMotorVoltage();
        currentSignal = motor.getTorqueCurrent();

        CANSignalManager.registerSignals(
            CANBUS,
            veloSignal, voltSignal, currentSignal);

        motor.optimizeBusUtilization(1.0);

        BootupLogger.bootupLog("    Shooter initialized (real)");
    }

    private TalonFXConfiguration motorConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = 0.2;
        cfg.Slot0.kV = 12.0 / kMotors.kFalcon500Foc.FREE_SPEED;
        cfg.Slot0.kS = 0.15;

        cfg.CurrentLimits.StatorCurrentLimit = PEAK_CURRENT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;
        cfg.Voltage.PeakReverseVoltage = 0.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return cfg;
    }

    public double getSpeed() {
        return radiansPerSecond;
    }

    public double getTargetSpeed() {
        return targetRadiansPerSecond;
    }

    public void setSpeed(double speedRadPerSec) {
        targetRadiansPerSecond = speedRadPerSec;
        double targetRotsRight = Units.radiansToRotations(targetRadiansPerSecond) / MECHANISM_RATIO;
        motor.setControl(controlReq.withVelocity(targetRotsRight));
    }

    public void setVoltageOut(double volts) {
        targetRadiansPerSecond = 0.0;
        motor.setControl(controlReqVolts.withOutput(volts));
    }

    @Override
    public void periodic() {
        radiansPerSecond = Units.rotationsToRadians(veloSignal.getValueAsDouble() * MECHANISM_RATIO);
        log("speed", radiansPerSecond);
        log("targetSpeed", targetRadiansPerSecond);
        log("volts", voltSignal.getValueAsDouble());
        log("current", currentSignal.getValueAsDouble());
    }
}
