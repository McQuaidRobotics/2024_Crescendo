package com.igknighters.subsystems.stem.telescope;

import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.math.MathUtil;

public class TelescopeDisabled implements Telescope {
    double currentMeters = StemPosition.STARTING.telescopeMeters;
    final double slewRate = (0.5 / 50.0) * 0.75;

    @Override
    public double getTelescopeMeters() {
        return currentMeters;
    }

    @Override
    public void setTelescopeMeters(double meters) {
        // var clampedTarget = MathUtil.clamp(meters, kTelescope.MIN_METERS, kTelescope.MAX_METERS);
        currentMeters = currentMeters + MathUtil.clamp(meters - currentMeters, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        double percentOut = volts / 12.0;
        double metersPerSecond = slewRate * percentOut;
        currentMeters += metersPerSecond;
    }

    @Override
    public boolean isFwdLimitSwitchHit() {
        return currentMeters >= kTelescope.MAX_METERS * 0.98;
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        return currentMeters <= kTelescope.MIN_METERS * 0.98;
    }

    @Override
    public boolean hasHomed() {
        return true;
    }
}
