package com.igknighters.subsystems.stem.wrist;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import edu.wpi.first.math.util.Units;
import monologue.Annotations.Log;

public abstract class Wrist extends Component {
    @Log.NT protected double radians;
    @Log.NT protected double targetRadians;
    @Log.NT protected double radiansPerSecond = 0.0;
    @Log.NT protected double volts = 0.0;
    @Log.NT protected double amps = 0.0;
    @Log.NT protected double temp = 0.0;

    public Wrist(double startingRadians) {
        this.radians = startingRadians;
        this.targetRadians = startingRadians;
    }

    @Override
    public String getPath() {
        return "Wrist";
    }

    public abstract void setWristRadians(double radians);

    public abstract double getWristRadians();

    /**
     * Moves the wrist to the target and returns if it has reached the target.
     * Meant to be used in a kind of polling loop to wait the mechanism to reach
     * the target.
     * 
     * @param radians      The target angle to move to
     * @param tolerancMult The multiplier to apply to the tolerance, higher mult
     *                     means more tolerance
     * @return If the mechanism has reached the target
     */
    public boolean target(double radians, double tolerancMult) {
        this.setWristRadians(radians);
        return isAt(radians, tolerancMult);
    }

    /**
     * Returns if the mechanism is within a tolerance of a certain angle.
     * @param radians The angle to check against
     * @param toleranceMult The multiplier to apply to the tolerance, higher mult
     *                    means more tolerance
     * @return If the mechanism is within the tolerance of the angle
     */
    public boolean isAt(double radians, double toleranceMult) {
        return Math.abs(this.getWristRadians() - radians) < ConstValues.kStem.kWrist.TARGET_TOLERANCE * toleranceMult;
    }

    static double mechanismRadsToMotorRots(Double radians) {
        return (Units.radiansToRotations(radians) - 0.328) / 0.00717;
    }

    static double mechanismRadsToMotorRads(Double radians) {
        return Units.rotationsToRadians(mechanismRadsToMotorRots(radians));
    }

    static double motorRotsToMechanismRads(double motorRots) {
        return Units.rotationsToRadians((0.00717*motorRots)+0.328);
    }

    static double motorRadsToMechanismRads(double motorRads) {
        return motorRotsToMechanismRads(Units.radiansToRotations(motorRads));
    }

    public void setCoast(boolean shouldBeCoasting) {
    }
}
