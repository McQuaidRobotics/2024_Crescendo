package com.igknighters.subsystems.stem.wrist;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import edu.wpi.first.math.util.Units;
import monologue.Annotations.Log;

public abstract class Wrist extends Component {
    @Log protected double radians;
    @Log protected double targetRadians;
    @Log protected double encoderRadians;
    @Log protected double radiansPerSecond = 0.0;
    @Log protected double volts = 0.0;
    @Log protected double amps = 0.0;

    public Wrist(double startingRadians) {
        this.radians = startingRadians;
        this.targetRadians = startingRadians;
        this.encoderRadians = startingRadians;
    }

    @Override
    public String getOverrideName() {
        return "Wrist";
    }

    /**
     * Commands the wrist to move towards a certain angle in radians.
     * 
     * @param radians The target angle to move to
     */
    public abstract void gotoPosition(double radians);

    /**
     * @return The current position of the wrist in radians
     */
    public double getPosition() {
        return this.radians;
    }

    /**
     * @return The current velocity of the wrist in radians per second
     */
    public double getVelocity() {
        return this.radiansPerSecond;
    }

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
        this.gotoPosition(radians);
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
        return Math.abs(this.getPosition() - radians) < ConstValues.kStem.kWrist.TARGET_TOLERANCE * toleranceMult;
    }

    public void setCoast(boolean shouldBeCoasting) {}

    /**
     * Runs the mechanism in open loop at the specified voltage
     * 
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public abstract void setVoltageOut(double volts);

    private static final double slope = 0.0075789;
    private static final double offset = 0.32939;

    protected static double mechanismRadsToMotorRots(Double radians) {
        return (Units.radiansToRotations(radians) - offset) / slope;
    }

    protected static double mechanismRadsToMotorRads(Double radians) {
        return Units.rotationsToRadians(mechanismRadsToMotorRots(radians));
    }

    protected static double motorRotsToMechanismRads(double motorRots) {
        return Units.rotationsToRadians((slope*motorRots)+offset);
    }

    protected static double motorRadsToMechanismRads(double motorRads) {
        return motorRotsToMechanismRads(Units.radiansToRotations(motorRads));
    }
}
