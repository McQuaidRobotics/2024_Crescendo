package com.igknighters.util;

public class MotionMagicProfileSim {
    final double allowedErrorPercent = 0.05;

    double cruiseVelo, targetAccel, targetJerk;
    double startingPosition, endingPosition, currentPosition;
    double velo, accel;

    public MotionMagicProfileSim(double cruiseVelo, double targetAccel, double targetJerk) {
        this.cruiseVelo = cruiseVelo;
        this.targetAccel = targetAccel;
        this.targetJerk = targetJerk;
        velo = 0.0;
        accel = 0.0;
        startingPosition = 0.0;
        endingPosition = 0.0;
    }

    public void setState(double velo, double currentPosition, double targetPosition) {
        this.velo = velo;
        startingPosition = currentPosition;
        this.currentPosition = currentPosition;
        endingPosition = targetPosition;
    }

    public double calculate() {
        return 0.0;
    }

    private boolean isAt(double value, double target, double allowedPercentError) {
        return Math.abs((value - target)) / target >= allowedPercentError;
    }

    private void moveAccelTowards(double targetAccel) {
        if (isAt(accel, targetAccel, allowedErrorPercent)) accel = targetAccel;
        else if (!isAt(accel, targetAccel, allowedErrorPercent) && accel < targetAccel) accel += targetJerk;
        else if (!isAt(accel, targetAccel, allowedErrorPercent) && accel > targetAccel) accel -= targetJerk;
    }

    private void moveVeloTowards(double targetVelo) {
        if (isAt(velo, targetVelo, allowedErrorPercent)) velo = cruiseVelo;
        else if (!isAt(velo, targetVelo, allowedErrorPercent) && velo < cruiseVelo) velo += accel;
        else if (!isAt(velo, targetVelo, allowedErrorPercent) && velo > cruiseVelo) velo -= accel;
    }

    public void update(double currentPosition) {
        double percentThroughMotion = (Math.abs(currentPosition) - Math.abs(startingPosition)) / (Math.abs(endingPosition) - Math.abs(startingPosition));
        
        //cruiseVelo = (jerk * x) * x 
        //cruiseVelo = jerk(x) + x^2 
        //cruiseVelo = 

        moveAccelTowards(targetAccel);
        moveVeloTowards(cruiseVelo);
    }
}
