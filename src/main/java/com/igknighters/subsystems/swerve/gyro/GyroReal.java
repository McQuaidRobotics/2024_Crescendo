package com.igknighters.subsystems.swerve.gyro;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.igknighters.constants.ConstValues;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroReal implements Gyro {

    private final Pigeon2 gyro;
    private final StatusSignal<Double> rollSignal, pitchSignal, yawSignal;
    private final StatusSignal<Double> rollVeloSignal, pitchVeloSignal, yawVeloSignal;

    private final GyroInputs inputs = new GyroInputs();

    public GyroReal() {
        gyro = new Pigeon2(ConstValues.kSwerve.PIGEON_ID, ConstValues.kSwerve.CANBUS);
        gyro.getConfigurator().apply(new Pigeon2Configuration());

        rollSignal = gyro.getRoll();
        pitchSignal = gyro.getPitch();
        yawSignal = gyro.getYaw();

        pitchVeloSignal = gyro.getAngularVelocityXDevice();
        rollVeloSignal = gyro.getAngularVelocityYDevice();
        yawVeloSignal = gyro.getAngularVelocityZDevice();

        rollSignal.setUpdateFrequency(100);
        pitchSignal.setUpdateFrequency(100);
        yawSignal.setUpdateFrequency(100);
        rollVeloSignal.setUpdateFrequency(100);
        pitchVeloSignal.setUpdateFrequency(100);
        yawVeloSignal.setUpdateFrequency(100);

        gyro.optimizeBusUtilization();
    }

    @Override
    public double getPitchRads() {
        return inputs.pitchRads;
    }

    @Override
    public double getRollRads() {
        return inputs.rollRads;
    }

    @Override
    public double getYawRads() {
        return inputs.yawRads;
    }

    @Override
    public void setYawRads(double yawRads) {
        gyro.setYaw(Units.radiansToDegrees(yawRads));
    }

    @Override
    public void setVoltageOut(double volts) {
        DriverStation.reportError("\"GyroSim.setVoltageOut\" is not a supported operation", false);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            pitchSignal, pitchVeloSignal,
            rollSignal, rollVeloSignal,
            yawSignal, yawVeloSignal
        );

        inputs.pitchRads = Units.degreesToRadians(pitchSignal.getValue());
        inputs.pitchVelRadsPerSec = Units.degreesToRadians(pitchVeloSignal.getValue());
        inputs.rollRads = Units.degreesToRadians(rollSignal.getValue());
        inputs.yawRads = Units.degreesToRadians(yawSignal.getValue());
        inputs.yawVelRadsPerSec = Units.degreesToRadians(yawVeloSignal.getValue());

        Logger.processInputs("Swerve/Gyro", inputs);
    }
}
