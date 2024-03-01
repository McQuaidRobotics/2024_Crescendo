package com.igknighters.subsystems.stem.wrist;

public class WristRealSuicidal extends WristReal {

    // final double the13thReason = (kWrist.MIN_ANGLE + kWrist.MAX_ANGLE) / 2.0;

    public static boolean sweetReleaseOfDeath = false;

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
}
