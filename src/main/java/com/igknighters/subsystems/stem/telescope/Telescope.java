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
        public boolean isLimitSwitchHit = false;

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
            table.put("isLimitSwitchHit", isLimitSwitchHit);

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
            isLimitSwitchHit = table.get("isLimitSwitchHit", isLimitSwitchHit);
        }
    }


}
