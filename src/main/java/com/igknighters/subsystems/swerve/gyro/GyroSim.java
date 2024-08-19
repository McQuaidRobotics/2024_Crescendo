package com.igknighters.subsystems.swerve.gyro;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Supplier;

public class GyroSim extends Gyro {

    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    public GyroSim(Supplier<ChassisSpeeds> chassisSpeedSupplier, SimSwerveOdometryThread odoThread) {
        this.chassisSpeedSupplier = chassisSpeedSupplier;

        odoThread.addRotationSupplier(() -> Rotation2d.fromRadians(this.getYawRads()));

        BootupLogger.bootupLog("    Gyro initialized (sim)");
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
        return super.yawRads;
    }

    @Override
    public void setYawRads(double yawRads) {
        super.yawRads = yawRads;
    }

    @Override
    public void periodic() {
        var oldYaw = super.yawRads;
        super.yawRads += chassisSpeedSupplier.get().omegaRadiansPerSecond * ConstValues.PERIODIC_TIME;
        super.yawVelRadsPerSec = (super.yawRads - oldYaw) / ConstValues.PERIODIC_TIME;
    }
}
