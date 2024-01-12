package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;

public interface Intake extends Component {

    public static class IntakeInputs implements LoggableInputs {
        public boolean hasGamePiece = false;
        public double radiansPerSecond = 0.0, volts = 0.0, amps = 0.0, temp = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("hasGamePiece", hasGamePiece);
            table.put("radiansPerSecond", radiansPerSecond);
            table.put("volts", volts);
            table.put("amps", amps);
            table.put("temp", temp);
        }

        @Override
        public void fromLog(LogTable table) {
            hasGamePiece = table.get("hasGamePiece", hasGamePiece);
            radiansPerSecond = table.get("radiansPerSecond", radiansPerSecond);
            volts = table.get("volts", volts);
            amps = table.get("amps", amps);
            temp = table.get("temp", temp);
        }
    }

    /**
     * @return If the {@code Intake} has a game piece
     */
    public boolean hasGamePiece();

    /**
     * Runs the {@code Intake} in open loop at the specified voltage
     * 
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public void setVoltageOut(double volts);

    /**
     * Turns the rotor on the {@code Intake} a specified number of radians
     * 
     * @param radians The number of radians to turn the rotor
     */
    public void turnIntake(double radians);
}
