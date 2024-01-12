package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;

public interface Intake extends Component {

    public static class IntakeInputs implements LoggableInputs {
        public boolean hasGamePiece = false;
        public double volts = 0.0, amps = 0.0, temp = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("hasGamePiece", hasGamePiece);
            table.put("volts", volts);
            table.put("amps", amps);
            table.put("temp", temp);
        }

        @Override
        public void fromLog(LogTable table) {
            hasGamePiece = table.get("hasGamePiece", hasGamePiece);
            volts = table.get("volts", volts);
            amps = table.get("amps", amps);
            temp = table.get("temp", temp);
        }
    }
}
