package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;

public interface Shooter extends Component {

    public static class ShooterInputs implements LoggableInputs {
        public double radiansPerSecond = 0.0, targetRadiansPerSecond = 0.0;
        public double volts = 0.0, amps = 0.0, temp = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("Umbrella/Shooter/radiansPerSecond", radiansPerSecond);
            table.put("Umbrella/Shooter/targetRadiansPerSecond", targetRadiansPerSecond);
            table.put("Umbrella/Shooter/volts", volts);
            table.put("Umbrella/Shooter/amps", amps);
            table.put("Umbrella/Shooter/temp", temp);
        }

        @Override
        public void fromLog(LogTable table) {
            radiansPerSecond = table.get("Umbrella/Shooter/radiansPerSecond", radiansPerSecond);
            targetRadiansPerSecond = table.get("Umbrella/Shooter/targetRadiansPerSecond", targetRadiansPerSecond);
            volts = table.get("Umbrella/Shooter/volts", volts);
            amps = table.get("Umbrella/Shooter/amps", amps);
            temp = table.get("Umbrella/Shooter/temp", temp);
        }
    }
}
