package com.igknighters.subsystems.stem.wrist;

import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.math.MathUtil;

public class WristDisabled implements Wrist {
    double targetRads = StemPosition.STARTING.wristRads;
    final double slewRate = (4.3 / 50.0) * 0.75;

    @Override
    public double getWristRadians() {
        return targetRads;
    }

    @Override
    public void setWristRadians(Double radians) {
        // var clampedTarget = MathUtil.clamp(radians, kWrist.MIN_ANGLE, kWrist.MAX_ANGLE);
        targetRads = targetRads + MathUtil.clamp(radians - targetRads, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {}
}
