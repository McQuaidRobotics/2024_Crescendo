package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import edu.wpi.first.math.util.Units;

public interface Intake extends Component {

    public static class IntakeInputs implements LoggableInputs {
        public boolean entranceBeamBroken = false, exitBeamBroken = false;
        public double radiansPerSecond = 0.0, volts = 0.0, amps = 0.0, temp = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("entranceBeamBroken", entranceBeamBroken);
            table.put("exitBeamBroken", exitBeamBroken);
            table.put("radiansPerSecond", radiansPerSecond);
            table.put("volts", volts);
            table.put("amps", amps);
            table.put("temp", temp);

            // A subtable, thats only written to when in debug mode and never read from,
            // that provides some more human readable values
            if (ConstValues.DEBUG) {
                table.put("#Human/rpm", Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond));
                table.put("#Human/targetRPM", Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond));
                table.put("#Human/watts", volts * amps);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            entranceBeamBroken = table.get("entranceBeamBroken", entranceBeamBroken);
            exitBeamBroken = table.get("exitBeamBroken", exitBeamBroken);
            radiansPerSecond = table.get("radiansPerSecond", radiansPerSecond);
            volts = table.get("volts", volts);
            amps = table.get("amps", amps);
            temp = table.get("temp", temp);
        }
    }

    /**
     * Turns the rotor on the {@code Intake} a specified number of radians
     * 
     * @param radians The number of radians to turn the rotor
     */
    public void turnIntakeRads(double radians);

    public default void turnIntakeMeters(double meters) {/* TODO */};

    /**
     * @return If the entrance beam is broken
     */
    public boolean isEntranceBeamBroken();

    /**
     * @return If the exit beam is broken
     */
    public boolean isExitBeamBroken();
}
