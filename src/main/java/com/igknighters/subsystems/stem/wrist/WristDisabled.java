package com.igknighters.subsystems.stem.wrist;

import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;

public class WristDisabled extends Wrist {
    final double slewRate = (4.3 / 50.0) * 0.75;

    public WristDisabled() {
        super(StemPosition.STARTING.wristRads);
    }

    @Override
    public double getWristRadians() {
        return super.radians;
    }

    @Override
    public void setWristRadians(double radians) {
        super.targetRadians = radians;
        super.radians = super.radians + MathUtil.clamp(radians - super.radians, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        super.volts = volts;
        double percentOut = volts / 12.0;
        super.radiansPerSecond = slewRate * percentOut;
        super.radians += super.radiansPerSecond;
    }

    @Override
    public void periodic() {}
}
