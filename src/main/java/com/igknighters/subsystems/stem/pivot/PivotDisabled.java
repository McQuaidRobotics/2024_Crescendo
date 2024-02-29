package com.igknighters.subsystems.stem.pivot;

import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.math.MathUtil;

public class PivotDisabled implements Pivot {
    double currentRads = StemPosition.STARTING.pivotRads;
    final double slewRate = (2.37 / 50.0) * 0.75;

    @Override
    public double getPivotRadians() {
        return currentRads;
    }

    @Override
    public void setPivotRadians(double radians) {
        // var clampedTarget = MathUtil.clamp(radians, kPivot.MIN_ANGLE, kPivot.MAX_ANGLE);
        currentRads = currentRads + MathUtil.clamp(radians - currentRads, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        double percentOut = volts / 12.0;
        double radiansPerSecond = slewRate * percentOut;
        currentRads += radiansPerSecond;
    }
}
