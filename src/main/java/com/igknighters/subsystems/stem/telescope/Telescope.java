package com.igknighters.subsystems.stem.telescope;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

public interface Telescope extends Component {

    public static class TelescopeInputs implements LoggableInputs {
        public double meters, targetMeters, metersPerSecond = 0.0;
        public double volts = 0.0;
        public double temp = 0.0, amps = 0.0;
        public boolean isLimitFwdSwitchHit = false, isLimitRevSwitchHit = false;
        public boolean isHomed = false;

        public TelescopeInputs(double startingMeters) {
            this.meters = startingMeters;
            this.targetMeters = startingMeters;
        }
 
        @Override
        public void toLog(LogTable table) {
            table.put("meters", meters);
            table.put("targetMeters", targetMeters);
            table.put("metersPerSecond", metersPerSecond);
            table.put("volts", volts);
            table.put("temp", temp);
            table.put("amps", amps);
            table.put("isLimitFwdSwitchHit", isLimitFwdSwitchHit);
            table.put("isLimitRevSwitchHit", isLimitRevSwitchHit);
            table.put("TelescopeHomed", isHomed);


            // A subtable, thats only written to when in debug mode and never read from,
            // that provides some more human readable values
            if (ConstValues.DEBUG) {
                table.put("#Human/watts", volts * amps);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            meters = table.get("meters", meters);
            targetMeters = table.get("targetMeters", targetMeters);
            metersPerSecond = table.get("metersPerSecond", metersPerSecond);
            volts = table.get("volts", volts);
            temp = table.get("temp", temp);
            amps = table.get("amps", amps);
            isLimitFwdSwitchHit = table.get("isLimitFwdSwitchHit", isLimitFwdSwitchHit);
            isLimitRevSwitchHit = table.get("isLimitRevSwitchHit", isLimitRevSwitchHit);
            isHomed = table.get("TelescopeHomed", isHomed);

        }
    }


    /**
     * @param meters The distance to set the mechanism to
     * 
     * @apiNote This is distance from pivot axel to the wrist axel,
     * this means 0.0 is not fully retraccted but rather an unreachable
     * position. Check {@link ConstValues.kStem.kTelescope.MIN_EXTENSION} and
     * {@link ConstValues.kStem.kTelescope.MAX_EXTENSION} for the min and max.
     */
    public void setTelescopeMeters(double meters);

    /**
     * @return The current distance from the pivot axel to the wrist axel
     */
    public double getTelescopeMeters();


    public boolean isFwdLimitSwitchHit();

    public boolean isRevLimitSwitchHit();

    /**
     * Move the telescope to the target and returns if it has reached the target.
     * Meant to be used in a kind of polling loop to wait the mechanism to reach
     * the target.
     * @param meters The target distance to move to
     * @param tolerancMult The multiplier to apply to the tolerance, higher mult means more tolerance
     * @return If the mechanism has reached the target
     */
    default public boolean target(double meters, double tolerancMult) {
        this.setTelescopeMeters(meters);
        return Math.abs(this.getTelescopeMeters() - meters) < ConstValues.kStem.kTelescope.TARGET_TOLERANCE * tolerancMult;
    }

    /**
     * Move the telescope to the target and returns if it has reached the target.
     * Meant to be used in a kind of polling loop to wait the mechanism to reach
     * the target.
     * @param meters The target distance to move to
     * @return If the mechanism has reached the target
     */
    default public boolean target(double meters) {
        return target(meters, 1.0);
    }

}
