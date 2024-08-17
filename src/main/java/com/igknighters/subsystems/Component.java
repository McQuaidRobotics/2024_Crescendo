package com.igknighters.subsystems;

import monologue.Logged;

public abstract class Component implements Logged {

    /**
     * Should be called every cycle in the parent subsystems periodic method
     */
    public void periodic() {};

    /**
     * Runs the mechanism in open loop at the specified voltage
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public void setVoltageOut(double volts) {
        throw new UnsupportedOperationException("setVoltageOut not implemented");
    }

    /**
     * Stops the mechanism
     */
    public void stopMechanism() {
        setVoltageOut(0.0);
    }
}
