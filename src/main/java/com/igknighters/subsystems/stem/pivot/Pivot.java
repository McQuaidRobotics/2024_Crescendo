package com.igknighters.subsystems.stem.pivot;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import monologue.Annotations.Log;

public abstract class Pivot extends Component {
    @Log.NT protected double radians;
    @Log.NT protected double targetRadians;
    @Log.NT protected double radiansPerSecond = 0.0;
    @Log.NT protected double leftVolts = 0.0;
    @Log.NT protected double rightVolts = 0.0;
    @Log.NT protected double leftAmps = 0.0;
    @Log.NT protected double rightAmps = 0.0;
    @Log.NT protected double leftTemp = 0.0;
    @Log.NT protected double rightTemp = 0.0;
    @Log.NT protected double gyroRadians = 0.0;
    @Log.NT protected boolean isLimitFwdSwitchHit = false, isLimitRevSwitchHit = false;

    protected Pivot(double startingRadians) {
        this.radians = startingRadians;
        this.targetRadians = startingRadians;
    }

    /**
     * @param radians the angle to set the mechanism to
     */
    public abstract void setPivotRadians(double radians);

    /**
     * @return the current angle of the mechanism
     */
    public abstract double getPivotRadians();

    /**
     * Move the pivot to the target and returns if it has reached the target.
     * Meant to be used in a kind of polling loop to wait the mechanism to reach
     * the target.
     * 
     * @param radians      The target angle to move to
     * @param tolerancMult The multiplier to apply to the tolerance, higher mult
     *                     means more tolerance
     * @return If the mechanism has reached the target
     */
    public boolean target(double radians, double tolerancMult) {
        this.setPivotRadians(radians);
        return isAt(radians, tolerancMult);
    }

    /**
     * Returns if the mechanism is within a tolerance of a certain angle.
     * 
     * @param radians       The angle to check against
     * @param toleranceMult The multiplier to apply to the tolerance, higher mult
     *                      means more tolerance
     * @return If the mechanism is within the tolerance of the angle
     */
    public boolean isAt(double radians, double toleranceMult) {
        return Math.abs(this.getPivotRadians() - radians) < ConstValues.kStem.kPivot.TARGET_TOLERANCE * toleranceMult;
    }

    public void setCoast(boolean shouldBeCoasting) {
    }
}
