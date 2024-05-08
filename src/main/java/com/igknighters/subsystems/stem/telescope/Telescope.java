package com.igknighters.subsystems.stem.telescope;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import monologue.Annotations.Log;

public abstract class Telescope extends Component {
    @Log.NT protected double meters;
    @Log.NT protected double targetMeters;
    @Log.NT protected double metersPerSecond = 0.0;
    @Log.NT protected double volts = 0.0;
    @Log.NT protected double temp = 0.0;
    @Log.NT protected double amps = 0.0;
    @Log.NT protected boolean isLimitFwdSwitchHit = false;
    @Log.NT protected boolean isLimitRevSwitchHit = false;
    @Log.NT protected boolean isHomed = false;

    protected Telescope(double startingMeters) {
        this.meters = startingMeters;
        this.targetMeters = startingMeters;
    }

    @Override
    public String getOverrideName() {
        return "Telescope";
    }

    /**
     * @param meters The distance to set the mechanism to
     * 
     * @apiNote This is distance from pivot axel to the wrist axel,
     *          this means 0.0 is not fully retraccted but rather an unreachable
     *          position. Check {@link ConstValues.kStem.kTelescope.MIN_METERS} and
     *          {@link ConstValues.kStem.kTelescope.MAX_METERS} for the min and max.
     */
    public abstract void setTelescopeMeters(double meters);

    /**
     * @return The current distance from the pivot axel to the wrist axel
     */
    public abstract double getTelescopeMeters();

    public abstract boolean isFwdLimitSwitchHit();

    public abstract boolean isRevLimitSwitchHit();

    public abstract boolean hasHomed();

    /**
     * Move the telescope to the target and returns if it has reached the target.
     * Meant to be used in a kind of polling loop to wait the mechanism to reach
     * the target.
     * 
     * @param meters       The target distance to move to
     * @param tolerancMult The multiplier to apply to the tolerance, higher mult
     *                     means more tolerance
     * @return If the mechanism has reached the target
     */
    public boolean target(double meters, double tolerancMult) {
        this.setTelescopeMeters(meters);
        return isAt(meters, tolerancMult);
    }

    /**
     * Returns if the mechanism is within a tolerance of a certain angle.
     * @param radians The angle to check against
     * @param toleranceMult The multiplier to apply to the tolerance, higher mult
     *                    means more tolerance
     * @return If the mechanism is within the tolerance of the angle
     */
    public boolean isAt(double radians, double toleranceMult) {
        return Math.abs(this.getTelescopeMeters() - radians) < ConstValues.kStem.kTelescope.TARGET_TOLERANCE * toleranceMult;
    }

    public void setCoast(boolean shouldBeCoasting) {
    }
}
