package com.igknighters.util;

public class MotionMagicProfileSim {
    double cruiseVelo, targetAccel, targetJerk;
    double velo, accel;

    public MotionMagicProfileSim(double cruiseVelo, double targetAccel, double targetJerk) {
        this.cruiseVelo = cruiseVelo;
        this.targetAccel = targetAccel;
        this.targetJerk = targetJerk;
        velo = 0.0;
        accel = 0.0;
    }

    private boolean isAt(double value, double target, double allowedPercentError) {
        return Math.abs((value - target)) / target > allowedPercentError;
    }

    public double calculate(double position) {
        return position + velo;
    }

    public void update() {
        if (!isAt(accel, targetAccel, 0.05)) accel += targetJerk;
        if (!isAt(velo, cruiseVelo, 0.05)) velo += accel;
    }
}
