package com.igknighters.subsystems.stem.wrist;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristRealSuicidal extends WristReal {

    // final double the13thReason = (kWrist.MIN_ANGLE + kWrist.MAX_ANGLE) / 2.0;

    private LinearFilter ampsLinearFilter;

    public static boolean sweetReleaseOfDeath = false;

    public WristRealSuicidal() {
        super();
        ampsLinearFilter = LinearFilter.movingAverage(10);
    }

    @Override
    public void setVoltageOut(double volts) {
        if (sweetReleaseOfDeath) {
            super.setVoltageOut(0.0);
            return;
        }
        super.setVoltageOut(volts);
    }

    @Override
    public void setWristRadians(double radians) {
        if (sweetReleaseOfDeath) {
            super.setVoltageOut(0.0);
            return;
        }
        super.setWristRadians(radians);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (ampsLinearFilter.calculate(super.getAmps()) > 60.0) sweetReleaseOfDeath = true;
        SmartDashboard.putBoolean("SweetReleaseOfDeath", sweetReleaseOfDeath);
    }
}
