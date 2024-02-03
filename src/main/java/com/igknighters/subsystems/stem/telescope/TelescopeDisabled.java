package com.igknighters.subsystems.stem.telescope;

import com.igknighters.constants.ConstValues.kStem.kTelescope;

public class TelescopeDisabled implements Telescope {
    double targetMeters = kTelescope.MIN_METERS;

    @Override
    public double getTelescopeMeters() {
        return targetMeters;
    }

    @Override
    public void setTelescopeMeters(double meters) {
        targetMeters = meters;
    }

    @Override
    public void setVoltageOut(double volts) {}

    @Override
    public boolean isFwdLimitSwitchHit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isFwdLimitSwitchHit'");
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isRevLimitSwitchHit'");
    }
}
