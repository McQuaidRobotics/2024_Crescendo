package com.igknighters.subsystems.stem.pivot;

import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

public class PivotDisabled extends Pivot {
    final double slewRate = (2.37 / 50.0) * 0.75;

    public PivotDisabled() {
        super(StemPosition.STARTING.pivotRads);
    }

    @Override
    public double getPivotRadians() {
        return super.radians;
    }

    @Override
    public void setPivotRadians(double radians) {
        super.radians = super.radians + MathUtil.clamp(radians - super.radians, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        double percentOut = volts / RobotController.getBatteryVoltage();
        super.radiansPerSecond = slewRate * percentOut;
        super.radians += super.radiansPerSecond;
    }
}
