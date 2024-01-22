package com.igknighters.subsystems;

public interface Component {

    default public void periodic() {};

    /**
     * Runs the mechanism in open loop at the specified voltage
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public void setVoltageOut(double volts);

    /**
     * Stops the mechanism
     */
    public default void stopMechanism() {
        setVoltageOut(0.0);
    }
}
