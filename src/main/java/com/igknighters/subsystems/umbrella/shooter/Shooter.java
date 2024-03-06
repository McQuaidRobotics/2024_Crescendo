package com.igknighters.subsystems.umbrella.shooter;

import com.igknighters.subsystems.Component;

import monologue.Annotations.Log;

public abstract class Shooter extends Component {

    @Log.NT protected double radiansPerSecondRight = 0.0;
    @Log.NT protected double targetRadiansPerSecondRight = 0.0;
    @Log.NT protected double radiansPerSecondLeft = 0.0;
    @Log.NT protected double targetRadiansPerSecondLeft = 0.0;
    @Log.NT protected double voltsRight = 0.0;
    @Log.NT protected double voltsLeft = 0.0;
    @Log.NT protected double ampsRight = 0.0;
    @Log.NT protected double ampsLeft = 0.0;
    @Log.NT protected double tempRight = 0.0;
    @Log.NT protected double tempLeft = 0.0;

    /**
     * @return The rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public abstract double getSpeed();

    /**
     * @return The target rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public abstract double getTargetSpeed();

    /**
     * Runs the {@code Shooter} in closed loop at the specified speed
     * 
     * @param speedRadPerSec The speed in Rad/S to spin the flywheel at
     */
    public abstract void setSpeed(double speedRadPerSec);
}
