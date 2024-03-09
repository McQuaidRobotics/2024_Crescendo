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

    private double sigmoid(double degree1, double degree2, double startingPos, double time) {
        return (time * (Math.pow((degree1 / degree2) + (time * time), -0.5)) * (degree1 - startingPos)) + startingPos; 
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
