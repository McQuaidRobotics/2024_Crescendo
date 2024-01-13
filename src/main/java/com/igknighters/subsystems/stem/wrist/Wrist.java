package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;

public interface Wrist extends Component {

    public static class WristInputs implements LoggableInputs {
        public double radians, targetRadians, radiansPerSecond = 0.0;
        public double volts = 0.0, amps = 0.0, temp = 0.0;
        public boolean isHomed = false;

        public WristInputs(double startingRadians) {
            this.radians = startingRadians;
            this.targetRadians = startingRadians;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("radians", radians);
            table.put("targetRadians", targetRadians);
            table.put("radiansPerSecond", radiansPerSecond);
            table.put("volts", volts);
            table.put("amps", amps);
            table.put("temp", temp);
        }

        @Override
        public void fromLog(LogTable table) {
            radians = table.get("radians", radians);
            targetRadians = table.get("targetRadians", targetRadians);
            radiansPerSecond = table.get("radiansPerSecond", radiansPerSecond);
            volts = table.get("volts", volts);
            amps = table.get("amps", amps);
            temp = table.get("temp", temp);
        }
    }

    public boolean setWristRadians (Double radians);

    public Double getWristRadians();

}
