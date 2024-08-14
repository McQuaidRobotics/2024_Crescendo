package com.igknighters.subsystems.umbrella.intake;

import com.igknighters.subsystems.Component;

import monologue.Annotations.Log;

public abstract class Intake extends Component {
    @Log.NT protected boolean exitBeamBroken = false;
    @Log.NT protected double radiansPerSecondUpper = 0.0;
    @Log.NT protected double voltsUpper = 0.0;
    @Log.NT protected double ampsUpper = 0.0;
    @Log.NT protected double radiansPerSecondLower = 0.0;
    @Log.NT protected double voltsLower = 0.0;
    @Log.NT protected double ampsLower = 0.0;

    @Override
    public String getOverrideName() {
        return "Intake";
    }

    /**
     * @return If the exit beam is broken
     */
    public abstract boolean isExitBeamBroken();

    /**
     * @return The output of the {@code Intake} in volts
     */
    public double getVoltageOut() {
        return voltsLower;
    }

    public abstract void setVoltageOut(double volts, boolean force);
}
