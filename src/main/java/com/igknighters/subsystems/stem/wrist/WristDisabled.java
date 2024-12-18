package com.igknighters.subsystems.stem.wrist;

import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

public class WristDisabled extends Wrist {
    final double slewRate = (4.3 / 50.0) * 0.5;

    public WristDisabled() {
        super(StemPosition.STARTING.wristRads);
    }

    @Override
    public void gotoPosition(double radians) {
        super.targetRadians = radians;
        super.radians = super.radians + MathUtil.clamp(radians - super.radians, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        super.volts = volts;
        double percentOut = volts / RobotController.getBatteryVoltage();
        super.radiansPerSecond = slewRate * percentOut;
        super.radians += super.radiansPerSecond;
    }

    @Override
    public void periodic() {}
}
