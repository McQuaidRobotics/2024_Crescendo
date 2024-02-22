package com.igknighters.subsystems.stem.pivot;

import com.igknighters.constants.ConstValues.kStem.kPivot;

import edu.wpi.first.math.MathUtil;

public class PivotDisabled implements Pivot {
    double targetRads = kPivot.MIN_ANGLE;
    final double slewRate = 2.37 / 50.0;

    @Override
    public double getPivotRadians() {
        return targetRads;
    }

    @Override
    public void setPivotRadians(double radians) {
        // var clampedTarget = MathUtil.clamp(radians, kPivot.MIN_ANGLE, kPivot.MAX_ANGLE);
        targetRads = targetRads + MathUtil.clamp(radians - targetRads, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {}
}
