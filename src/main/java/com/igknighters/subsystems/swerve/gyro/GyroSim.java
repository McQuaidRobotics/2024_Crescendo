package com.igknighters.subsystems.swerve.gyro;

import com.igknighters.constants.ConstValues;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class GyroSim implements Gyro {

    private final GyroInputs inputs = new GyroInputs();

    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    public GyroSim(Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.chassisSpeedSupplier = chassisSpeedSupplier;
    }

    @Override
    public double getPitchRads() {
        return 0.0;
    }

    @Override
    public double getRollRads() {
        return 0.0;
    }

    @Override
    public double getYawRads() {
        return inputs.yawRads;
    }

    @Override
    public void setYawRads(double yawRads) {
        inputs.yawRads = yawRads;
    }

    @Override
    public void setVoltageOut(double volts) {
        DriverStation.reportError("\"GyroSim.setVoltageOut\" is not a supported operation", false);
    }

    @Override
    public void periodic() {
        var oldYaw = inputs.yawRads;
        inputs.yawRads += chassisSpeedSupplier.get().omegaRadiansPerSecond * ConstValues.PERIODIC_TIME;
        inputs.yawVelRadsPerSec = (inputs.yawRads - oldYaw) / ConstValues.PERIODIC_TIME;

        Logger.processInputs("Swerve/Gyro", inputs);
    }
}
