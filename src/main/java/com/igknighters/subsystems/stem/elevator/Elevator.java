package com.igknighters.subsystems.stem.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;

public interface Elevator extends Component {

    public static class ElevatorInputs implements LoggableInputs {
        public double inches, targetInches, inchesPerSecond = 0.0;
        public double volts = 0.0;
        public double temp = 0.0, amps = 0.0;
        public boolean isLimitSwitchHit = false;

        public ElevatorInputs(double startingInches) {
            this.inches = startingInches;
            this.targetInches = startingInches;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("Stem/Elevator/inches", inches);
            table.put("Stem/Elevator/targetInches", targetInches);
            table.put("Stem/Elevator/inchesPerSecond", inchesPerSecond);
            table.put("Stem/Elevator/volts", volts);
            table.put("Stem/Elevator/temp", temp);
            table.put("Stem/Elevator/amps", amps);
            table.put("Stem/Elevator/isLimitSwitchHit", isLimitSwitchHit);
        }

        @Override
        public void fromLog(LogTable table) {
            inches = table.get("Stem/Elevator/inches", inches);
            targetInches = table.get("Stem/Elevator/targetInches", targetInches);
            inchesPerSecond = table.get("Stem/Elevator/inchesPerSecond", inchesPerSecond);
            volts = table.get("Stem/Elevator/volts", volts);
            temp = table.get("Stem/Elevator/temp", temp);
            amps = table.get("Stem/Elevator/amps", amps);
            isLimitSwitchHit = table.get("Stem/Elevator/isLimitSwitchHit", isLimitSwitchHit);
        }
    }


}
