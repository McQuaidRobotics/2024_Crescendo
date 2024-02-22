package com.igknighters.subsystems.stem.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

public interface Pivot extends Component {

    public static class PivotInputs implements LoggableInputs {
        public double radians, targetRadians, radiansPerSecond = 0.0;
        public double leftVolts = 0.0, rightVolts = 0.0;
        public double leftAmps = 0.0, rightAmps = 0.0;
        public double leftTemp = 0.0, rightTemp = 0.0;
        public double gyroRadians = 0.0;
        public boolean isLimitFwdSwitchHit = false, isLimitRevSwitchHit = false;

        public PivotInputs(double startingRadians) {
            this.radians = startingRadians;
            this.targetRadians = startingRadians;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("radians", radians);
            table.put("targetRadians", targetRadians);
            table.put("radiansPerSecond", radiansPerSecond);
            table.put("leftVolts", leftVolts);
            table.put("rightVolts", rightVolts);
            table.put("leftAmps", leftAmps);
            table.put("rightAmps", rightAmps);
            table.put("leftTemp", leftTemp);
            table.put("rightTemp", rightTemp);
            table.put("gyroRadians", gyroRadians);
            table.put("isLimitFwdSwitchHit", isLimitFwdSwitchHit);
            table.put("isLimitRevSwitchHit", isLimitRevSwitchHit);

            // A subtable, thats only written to when in debug mode and never read from,
            // that provides some more human readable values
            if (ConstValues.DEBUG) {
                table.put("#Human/degrees", Math.toDegrees(radians));
                table.put("#Human/gyroDegrees", Math.toDegrees(gyroRadians));
                table.put("#Human/targetDegrees", Math.toDegrees(targetRadians));
                table.put("#Human/degreesPerSecond", Math.toDegrees(radiansPerSecond));
            }
        }

        @Override
        public void fromLog(LogTable table) {
            radians = table.get("radians", radians);
            targetRadians = table.get("targetRadians", targetRadians);
            radiansPerSecond = table.get("radiansPerSecond", radiansPerSecond);
            leftVolts = table.get("leftVolts", leftVolts);
            rightVolts = table.get("rightVolts", rightVolts);
            leftAmps = table.get("leftAmps", leftAmps);
            rightAmps = table.get("rightAmps", rightAmps);
            leftTemp = table.get("leftTemp", leftTemp);
            rightTemp = table.get("rightTemp", rightTemp);
            gyroRadians = table.get("gyroRadians", gyroRadians);
            isLimitFwdSwitchHit = table.get("isLimitFwdSwitchHit", isLimitFwdSwitchHit);
            isLimitRevSwitchHit = table.get("isLimitRevSwitchHit", isLimitRevSwitchHit);
        }
    }

    /**
     * @param radians the angle to set the mechanism to
     */
    public void setPivotRadians(double radians);

    /**
     * @return the current angle of the mechanism
     */
    public double getPivotRadians();

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
    default public boolean target(double radians, double tolerancMult) {
        this.setPivotRadians(radians);
        return isAt(radians, tolerancMult);
    }

    /**
     * Returns if the mechanism is within a tolerance of a certain angle.
     * @param radians The angle to check against
     * @param toleranceMult The multiplier to apply to the tolerance, higher mult
     *                    means more tolerance
     * @return If the mechanism is within the tolerance of the angle
     */
    default public boolean isAt(double radians, double toleranceMult) {
        return Math.abs(this.getPivotRadians() - radians) < ConstValues.kStem.kPivot.TARGET_TOLERANCE * toleranceMult;
    }

    default public void setCoast(boolean shouldBeCoasting) {}
}
