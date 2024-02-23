package com.igknighters.subsystems.swerve;

import com.igknighters.constants.ConstValues;

public class RotationalController {
    private final double kP, kD;
    private final double maxVelocity, maxAcceleration;
    private double positionError = 0, prevError = 0, velocityError = 0;

    public RotationalController(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
        this.kP = kP;
        this.kD = kD;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public double calculate(double setpoint, double measurement) {
        prevError = positionError;

        double input = setpoint - measurement;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input + Math.PI) / (Math.PI * 2.0));
        input -= numMax * (Math.PI * 2.0);

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - Math.PI) / (Math.PI * 2.0));
        input -= numMin * (Math.PI * 2.0);

        velocityError = (positionError - prevError) / ConstValues.PERIODIC_TIME;

        return (kP * positionError) + (kD * velocityError);
    }
}
