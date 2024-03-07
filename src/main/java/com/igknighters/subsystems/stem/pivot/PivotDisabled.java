package com.igknighters.subsystems.stem.pivot;

import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

public class PivotDisabled extends Pivot {
    final double maxVelo = (2.37 / 50.0) * 0.75;
    final double maxAcc = 0.0001;
    final double maxJerk = 0.0001;
    double velo;
    double acc;

    public PivotDisabled() {
        super(StemPosition.STARTING.pivotRads);
        this.velo = 0.0;
        this.acc = 0.0;
    }

    @Override
    public double getPivotRadians() {
        return super.radians;
    }

    @Override
    public void setPivotRadians(double radians) {
        // double velo = MathUtil.clamp(radians - super.radians, -maxVelo, maxVelo);

        if (super.targetRadians != radians) velo = 0.0;
        super.targetRadians = radians;

        acc += maxJerk;
        acc = MathUtil.clamp(acc, -maxAcc, maxAcc);
        velo += acc;
        super.radians += MathUtil.clamp(velo, -maxVelo, maxVelo) * Math.signum(radians - super.radians);
    }

    @Override
    public void setVoltageOut(double volts) {
        double percentOut = volts / RobotController.getBatteryVoltage();
        super.radiansPerSecond = maxVelo * percentOut;
        super.radians += super.radiansPerSecond;
    }
}
