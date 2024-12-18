package com.igknighters.subsystems.stem.telescope;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import monologue.Annotations.Log;

public abstract class Telescope extends Component {
    @Log protected double meters;
    @Log protected double targetMeters;
    @Log protected double metersPerSecond = 0.0;
    @Log protected double volts = 0.0;
    @Log protected double temp = 0.0;
    @Log protected double amps = 0.0;
    @Log protected boolean isLimitFwdSwitchHit = false;
    @Log protected boolean isLimitRevSwitchHit = false;
    @Log protected boolean isHomed = false;

    protected Telescope(double startingMeters) {
        this.meters = startingMeters;
        this.targetMeters = startingMeters;
    }

    @Override
    public String getOverrideName() {
        return "Telescope";
    }

    /**
     * Commands the telescope to move towards a certain distance in meters.
     * 
     * @param meters The target distance to move to
     * 
     * @apiNote This is distance from pivot axel to the wrist axel,
     *          this means 0.0 is not fully retraccted but rather an unreachable
     *          position. Between {@value ConstValues.kStem.kTelescope.MIN_METERS} and
     *          {@value ConstValues.kStem.kTelescope.MAX_METERS} for the min and max.
     */
    public abstract void gotoPosition(double meters);

    /**
     * @return The current distance from the pivot axel to the wrist axel in meters
     */
    public double getPosition() {
        return this.meters;
    }

    /**
     * @return The current velocity of the mechanism in meters per second
     */
    public double getVelocity() {
        return this.metersPerSecond;
    }

    /**
     * @return If the forward limit switch is currently being triggered
     */
    public abstract boolean isFwdLimitSwitchHit();

    /**
     * @return If the reverse limit switch is currently being triggered
     */
    public abstract boolean isRevLimitSwitchHit();

    /**
     * @return If the mechanism has homed since bootup
     */
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
        this.gotoPosition(meters);
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
        return Math.abs(this.getPosition() - radians) < ConstValues.kStem.kTelescope.TARGET_TOLERANCE * toleranceMult;
    }

    /**
     * Sets the mechanism to coast or brake mode.
     * 
     * @param shouldBeCoasting If the mechanism should be coasting
     */
    public void setCoast(boolean shouldBeCoasting) {
        // provide a no-op implementation to make sim classes less verbose
    }

    /**
     * Runs the mechanism in open loop at the specified voltage
     * 
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public abstract void setVoltageOut(double volts);
}
