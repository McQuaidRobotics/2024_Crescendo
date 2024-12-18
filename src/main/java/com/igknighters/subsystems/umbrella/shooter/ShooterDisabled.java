package com.igknighters.subsystems.umbrella.shooter;

import com.igknighters.constants.ConstValues;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

public class ShooterDisabled extends Shooter {
    double currentRadPerSec = 0.0;
    double targetRadPerSec = 0.0;
    final double slewRateRate = 6380 / 1.2;

    public ShooterDisabled() {
    }

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
        setSpeed(MathUtil.clamp(volts / RobotController.getBatteryVoltage(), -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage()));
    }

    @Override
    public void periodic() {
        double maxDiff = slewRateRate * ConstValues.PERIODIC_TIME;
        double diff = MathUtil.clamp(
                targetRadPerSec - currentRadPerSec,
                -maxDiff,
                maxDiff);
        currentRadPerSec += diff;

        this.radiansPerSecondLeft = currentRadPerSec;
        this.radiansPerSecondRight = currentRadPerSec;

        this.targetRadiansPerSecondLeft = targetRadPerSec;
        this.targetRadiansPerSecondRight = targetRadPerSec;
    }
}
