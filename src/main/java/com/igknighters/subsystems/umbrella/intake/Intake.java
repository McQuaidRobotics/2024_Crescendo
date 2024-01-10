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
            table.put("Umbrella/Intake/hasGamePiece", hasGamePiece);
            table.put("Umbrella/Intake/volts", volts);
            table.put("Umbrella/Intake/amps", amps);
            table.put("Umbrella/Intake/temp", temp);
        }

        @Override
        public void fromLog(LogTable table) {
            hasGamePiece = table.get("Umbrella/Intake/hasGamePiece", hasGamePiece);
            volts = table.get("Umbrella/Intake/volts", volts);
            amps = table.get("Umbrella/Intake/amps", amps);
            temp = table.get("Umbrella/Intake/temp", temp);
        }
    }
}
