package com.igknighters.subsystems.stem.wrist;

import com.igknighters.constants.ConstValues.kStem.kWrist;

import edu.wpi.first.math.MathUtil;

public class WristDisabled implements Wrist {
    double targetRads = kWrist.MIN_ANGLE;
    final double slewRate = 1.2 / 50.0;

    @Override
    public double getWristRadians() {
        return targetRads;
    }

    @Override
    public void setWristRadians(Double radians) {
        var clampedTarget = MathUtil.clamp(radians, kWrist.MIN_ANGLE, kWrist.MAX_ANGLE);
        targetRads = targetRads + MathUtil.clamp(clampedTarget - targetRads, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {}
}
