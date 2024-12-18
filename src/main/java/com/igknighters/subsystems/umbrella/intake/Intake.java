package com.igknighters.subsystems.umbrella.intake;

import com.igknighters.subsystems.Component;

import monologue.Annotations.Log;

public abstract class Intake extends Component {
    @Log protected boolean exitBeamBroken = false;
    @Log protected double radiansPerSecondUpper = 0.0;
    @Log protected double voltsUpper = 0.0;
    @Log protected double ampsUpper = 0.0;
    @Log protected double radiansPerSecondLower = 0.0;
    @Log protected double voltsLower = 0.0;
    @Log protected double ampsLower = 0.0;

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

    /**
     * Runs the mechanism in open loop at the specified voltage
     * 
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public abstract void setVoltageOut(double volts);

    /**
     * Runs the mechanism in open loop at the specified voltage
     * 
     * @param volts The specified volts: [-12.0 .. 12.0]
     * @param force If the mechanism should force past the limit switches
     */
    public abstract void setVoltageOut(double volts, boolean force);
}
