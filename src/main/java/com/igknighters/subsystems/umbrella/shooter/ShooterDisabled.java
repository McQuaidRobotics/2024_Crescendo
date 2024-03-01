package com.igknighters.subsystems.umbrella.shooter;

import com.igknighters.constants.ConstValues;

import edu.wpi.first.math.MathUtil;

public class ShooterDisabled implements Shooter {
    double currentRadPerSec = 0.0;
    double targetRadPerSec = 0.0;
    final double slewRateRate = 6380 / 1.2;

    final ShooterInputs inputs = new ShooterInputs();

    public ShooterDisabled() {}

    @Override
    public void setSpeed(double speedRadPerSec) {
        targetRadPerSec = speedRadPerSec;
    }

    @Override
    public double getTargetSpeed() {
        return targetRadPerSec;
    }

    @Override
    public double getSpeed() {
        return currentRadPerSec;
    }

    @Override
    public void setVoltageOut(double volts) {
        setSpeed(MathUtil.clamp(volts / 12.0, -12.0, 12.0));
    }

    @Override
    public void stopMechanism() {
        targetRadPerSec = 0.0;
        currentRadPerSec = 0.0;
    }

    @Override
    public void periodic() {
        double maxDiff = slewRateRate * ConstValues.PERIODIC_TIME;
        double diff = MathUtil.clamp(
            targetRadPerSec - currentRadPerSec,
            -maxDiff,
            maxDiff
        );
        currentRadPerSec += diff;

        inputs.radiansPerSecondLeft = currentRadPerSec;
        inputs.radiansPerSecondRight = currentRadPerSec;

        inputs.targetRadiansPerSecondLeft = targetRadPerSec;
        inputs.targetRadiansPerSecondRight = targetRadPerSec;
    }
}
