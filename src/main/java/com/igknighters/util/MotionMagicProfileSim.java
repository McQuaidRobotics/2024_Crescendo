package com.igknighters.util;

import edu.wpi.first.wpilibj.Timer;

public class MotionMagicProfileSim {
    double cruiseVelo, targetAccel, targetJerk;
    double startingVelo, startingAccel;
    double startingPos, endingPos;
    Timer timer;

    public MotionMagicProfileSim(double cruiseVelo, double targetAccel, double targetJerk, double gearRation) {
        setMotionProfile(cruiseVelo / gearRation, targetAccel / gearRation, targetJerk / gearRation);
        startingPos = 0.0;
        endingPos = 0.0;
        startingVelo = 0.0;
        startingAccel = 0.0;
        timer = new Timer();
    }

    public void setMotionProfile(double cruiseVelo, double targetAccel, double targetJerk) {
        this.cruiseVelo = cruiseVelo;
        this.targetAccel = targetAccel;
        this.targetJerk = targetJerk;
    }

    private double sigmoid(double startPosition, double endPosition, double rateOfChange, double time) {
        double limit = 0.001;
        double horizontalOffset = (1.0 / -rateOfChange) * Math.log(((endPosition * rateOfChange) - (2.0 * limit) - Math.sqrt(Math.pow((2.0 * limit) - (endPosition * rateOfChange), 2) - (4 * Math.pow(limit, 2)))) / (2.0 * limit));
        return ((endPosition - startPosition) / (1.0 + Math.pow(Math.E, -rateOfChange * (time - horizontalOffset)))) + startPosition;
    }

    public double calculatePosition() {
        double time = timer.get();
        double accel = Math.abs(sigmoid(targetAccel, targetJerk, startingAccel, time));
        double velo = Math.abs(sigmoid(cruiseVelo, accel, startingVelo, time));
        return sigmoid(Math.abs(endingPos), velo, startingPos, time) * Math.signum(endingPos);
    }

    public double calculateVelocity() {
        double time = timer.get();
        double accel = Math.abs(sigmoid(targetAccel, targetJerk, startingAccel, time));
        return sigmoid(cruiseVelo, accel, startingVelo, time);
    }

    public double calculateAcceleration() {
        double time = timer.get();
        return sigmoid(targetAccel, targetJerk, startingAccel, time);
    }

    public void setState(double currentPosition, double targetPosition, double startingVelo, double startingAccel) {
        startingPos = currentPosition;
        endingPos = targetPosition;
        this.startingVelo = startingVelo;
        this.startingAccel = startingAccel;
        timer.restart();
    }
}
