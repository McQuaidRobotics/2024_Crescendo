package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

public interface Shooter extends Component {

    public static class ShooterInputs implements LoggableInputs {
        public double radiansPerSecond = 0.0, targetRadiansPerSecond = 0.0;
        public double volts = 0.0, amps = 0.0, temp = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("radiansPerSecond", radiansPerSecond);
            table.put("targetRadiansPerSecond", targetRadiansPerSecond);
            table.put("volts", volts);
            table.put("amps", amps);
            table.put("temp", temp);

            // A subtable, thats only written to when in debug mode and never read from,
            // that provides some more human readable values
            if (ConstValues.DEBUG) {
                table.put("#Human/degreesPerSecond", Math.toDegrees(radiansPerSecond));
                table.put("#Human/targetDegreesPerSecond", Math.toDegrees(targetRadiansPerSecond));
                table.put("#Human/watts", volts * amps);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            radiansPerSecond = table.get("radiansPerSecond", radiansPerSecond);
            targetRadiansPerSecond = table.get("targetRadiansPerSecond", targetRadiansPerSecond);
            volts = table.get("volts", volts);
            amps = table.get("amps", amps);
            temp = table.get("temp", temp);
        }
    }

    /**
     * @return The rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public double getSpeed();

    /**
     * Runs the {@code Shooter} in closed loop at the specified speed
     * @param speedRadPerSec The speed in Rad/S to spin the flywheel at
     */
    public void setSpeed(double speedRadPerSec);

    /**
     * Runs the {@code Shooter} flywheel in open loop at the specified voltage
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public void setVoltageOut(double volts);
}
