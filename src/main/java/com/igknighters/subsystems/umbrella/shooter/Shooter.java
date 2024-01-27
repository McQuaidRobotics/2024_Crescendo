package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import edu.wpi.first.math.util.Units;

public interface Shooter extends Component {

    public static class ShooterInputs implements LoggableInputs {
        public double radiansPerSecondUpper = 0.0, targetRadiansPerSecondUpper = 0.0;
        public double radiansPerSecondLower = 0.0, targetRadiansPerSecondLower = 0.0;
        public double voltsUpper = 0.0, voltsLower = 0.0;
        public double ampsUpper = 0.0, ampsLower = 0.0;
        public double tempUpper = 0.0, tempLower = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("radiansPerSecondUpper", radiansPerSecondUpper);
            table.put("targetRadiansPerSecondUpper", targetRadiansPerSecondUpper);
            table.put("radiansPerSecondLower", radiansPerSecondLower);
            table.put("targetRadiansPerSecondLower", targetRadiansPerSecondLower);
            table.put("ampsUpper", ampsUpper);
            table.put("ampsLower", ampsLower);
            table.put("voltsUpper", voltsUpper);
            table.put("voltsLower", voltsLower);
            table.put("tempUpper", tempUpper);
            table.put("tempLower", tempLower);

            // A subtable, thats only written to when in debug mode and never read from,
            // that provides some more human readable values
            if (ConstValues.DEBUG) {
                table.put("#Human/rpmUpper", Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondUpper));
                table.put("#Human/targetRPMUpper",
                        Units.radiansPerSecondToRotationsPerMinute(targetRadiansPerSecondUpper));
            }
        }

        @Override
        public void fromLog(LogTable table) {
            radiansPerSecondUpper = table.get("radiansPerSecond", radiansPerSecondUpper);
            targetRadiansPerSecondUpper = table.get("targetRadiansPerSecond", targetRadiansPerSecondUpper);
            radiansPerSecondLower = table.get("radiansPerSecond", radiansPerSecondLower);
            targetRadiansPerSecondLower = table.get("targetRadiansPerSecond", targetRadiansPerSecondLower);
            ampsUpper = table.get("ampsUpper", ampsUpper);
            ampsLower = table.get("ampsLower", ampsLower);
            voltsUpper = table.get("voltsUpper", voltsUpper);
            voltsLower = table.get("voltsLower", voltsLower);
            tempUpper = table.get("tempUpper", tempUpper);
            tempLower = table.get("tempLower", tempLower);
        }
    }

    /**
     * @return The rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public double getSpeed();

    /**
     * @return The target rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public double getTargetSpeed();

    /**
     * Runs the {@code Shooter} in closed loop at the specified speed
     * 
     * @param speedRadPerSec The speed in Rad/S to spin the flywheel at
     */
    public void setSpeed(double speedRadPerSec);
}
