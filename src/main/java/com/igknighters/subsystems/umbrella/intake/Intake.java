package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;

public interface Intake extends Component {

    public static class IntakeInputs implements LoggableInputs {
        public boolean entranceBeamBroken = false, exitBeamBroken = false;
        public double radiansPerSecondUpper = 0.0, voltsUpper = 0.0, ampsUpper = 0.0, tempUpper = 0.0;
        public double radiansPerSecondLower = 0.0, voltsLower = 0.0, ampsLower = 0.0, tempLower = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("entranceBeamBroken", entranceBeamBroken);
            table.put("exitBeamBroken", exitBeamBroken);
            table.put("radiansPerSecondUpper", radiansPerSecondUpper);
            table.put("voltsUpper", voltsUpper);
            table.put("ampsUpper", ampsUpper);
            table.put("tempUpper", tempUpper);
            table.put("radiansPerSecondLower", radiansPerSecondLower);
            table.put("voltsLower", voltsLower);
            table.put("ampsLower", ampsLower);
            table.put("tempLower", tempLower);
        }

        @Override
        public void fromLog(LogTable table) {
            entranceBeamBroken = table.get("entranceBeamBroken", entranceBeamBroken);
            exitBeamBroken = table.get("exitBeamBroken", exitBeamBroken);
            radiansPerSecondUpper = table.get("radiansPerSecondUpper", radiansPerSecondUpper);
            voltsUpper = table.get("voltsUpper", voltsUpper);
            ampsUpper = table.get("ampsUpper", ampsUpper);
            tempUpper = table.get("tempUpper", tempUpper);
            radiansPerSecondLower = table.get("radiansPerSecondLower", radiansPerSecondLower);
            voltsLower = table.get("voltsLower", voltsLower);
            ampsLower = table.get("ampsLower", ampsLower);
            tempLower = table.get("tempLower", tempLower);
        }
    }

    /**
     * Turns the rotor on the {@code Intake} a specified number of radians
     * 
     * @param radians The number of radians to turn the rotor
     */
    public void turnIntakeRads(double radians);

    /**
     * @return If the entrance beam is broken
     */
    public boolean isEntranceBeamBroken();

    /**
     * @return If the exit beam is broken
     */
    public boolean isExitBeamBroken();
}
