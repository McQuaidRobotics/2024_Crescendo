package com.igknighters.subsystems;

import com.igknighters.util.ShuffleboardApi.ShuffleEntryContainer;

public interface Component {

    default public void periodic() {};

     default public void setupShuffleboard(ShuffleEntryContainer tab) {
    };

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param percentOut of the mechanisms motors
     */
    public void manualDriveMechanism(Double percentOut);

    /**
     * Stops the mechanism
     */
    public void stopMechanism();

    /**
     * Moves the mechanism towards the home position,
     * @param force if true, will ignore if the mechanism is already at the home position
     * @return true if the mechanism has reached home
     */
    public boolean homeMechanism(boolean force);
}
