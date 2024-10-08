package com.igknighters.subsystems.rookie;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.igknighters.subsystems.SubsystemResources.LockFullSubsystem;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableDouble;

import edu.wpi.first.wpilibj.DriverStation;

public class Acquisition implements LockFullSubsystem {

    private static final int ROLLER_MOTOR_ID = 12;
    private static final int ARM_MOTOR_ID = 13;
    private static final String CANBUS = "rio";
    private static final double BACK_HARDSTOP_POSITION = 0.0;
    private static final double ARM_MECHANISM_RATIO = 1.0;

    private final TalonFX rollerMotor = new TalonFX(ROLLER_MOTOR_ID, CANBUS);
    private final TalonFX armMotor = new TalonFX(ARM_MOTOR_ID, CANBUS);

    private final StatusSignal<Double> voltSignalRoller, ampSignalRoller;
    private final StatusSignal<Double> voltSignalArm, ampSignalArm, posSignalArm;

    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final PositionVoltage controlReqPosition = new PositionVoltage(0.0).withUpdateFreqHz(0).withEnableFOC(true);

    private final TunableDouble currentTripValue = TunableValues.getDouble("AcquisitionRollerCurrentTrip", 115.0);

    public Acquisition() {
        rollerMotor.getConfigurator().apply(rollerMotorCfg());
        armMotor.getConfigurator().apply(armMotorCfg());

        voltSignalRoller = rollerMotor.getMotorVoltage();
        ampSignalRoller = rollerMotor.getTorqueCurrent();

        voltSignalArm = armMotor.getMotorVoltage();
        ampSignalArm = armMotor.getTorqueCurrent();
        posSignalArm = armMotor.getPosition();

        CANSignalManager.registerSignals(
            CANBUS,
            voltSignalRoller,
            voltSignalArm,
            posSignalArm
        );
        ampSignalRoller.setUpdateFrequency(200);
        ampSignalArm.setUpdateFrequency(200);

        rollerMotor.optimizeBusUtilization(1.0);

        BootupLogger.bootupLog("    Intake initialized (real)");
    }

    private TalonFXConfiguration rollerMotorCfg() {
        var cfg = new TalonFXConfiguration();

        cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = false;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentThreshold = 65.0;
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        cfg.CurrentLimits.SupplyTimeThreshold = 0.3;

        return cfg;
    }

    private TalonFXConfiguration armMotorCfg() {
        var cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = 1.0;
        cfg.Slot0.kS = 0.2;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 70.0;

        return cfg;
    }

    public void setRollerVoltageOut(double volts) {
        rollerMotor.setControl(controlReqVolts.withOutput(volts));
    }

    public void setArmPosition(double position) {
        armMotor.setControl(controlReqPosition.withPosition(position));
    }

    public void setArmVoltageOut(double volts) {
        armMotor.setControl(controlReqVolts.withOutput(volts));
    }

    public boolean isRollerCurrentTripped() {
        return ampSignalRoller.refresh().getValueAsDouble() > currentTripValue.value();
    }

    public double getArmPosition() {
        return posSignalArm.getValueAsDouble();
    }

    public boolean isArmCurrentTripped() {
        return ampSignalArm.refresh().getValueAsDouble() > 60.0;
    }

    public void homeArmHere() {
        armMotor.setPosition(BACK_HARDSTOP_POSITION / ARM_MECHANISM_RATIO);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            armMotor.setControl(controlReqVolts.withOutput(0.0));
        }

        BaseStatusSignal.refreshAll(
            ampSignalRoller,
            ampSignalArm,
            posSignalArm
        );

        log("rollerVolts", voltSignalRoller.getValueAsDouble());
        log("rollerAmps", ampSignalRoller.getValueAsDouble());
        log("armVolts", voltSignalArm.getValueAsDouble());
        log("armAmps", ampSignalArm.getValueAsDouble());
        log("armPos", posSignalArm.getValueAsDouble());
    }
}
