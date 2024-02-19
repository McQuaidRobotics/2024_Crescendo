package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import edu.wpi.first.math.util.Units;

public interface Shooter extends Component {

    public static class ShooterInputs implements LoggableInputs {
        public double radiansPerSecondRight = 0.0, targetRadiansPerSecondRight = 0.0;
        public double radiansPerSecondLeft = 0.0, targetRadiansPerSecondLeft = 0.0;
        public double voltsRight = 0.0, voltsLeft = 0.0;
        public double ampsRight = 0.0, ampsLeft = 0.0;
        public double tempRight = 0.0, tempLeft = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("radiansPerSecondRight", radiansPerSecondRight);
            table.put("targetRadiansPerSecondRight", targetRadiansPerSecondRight);
            table.put("radiansPerSecondLeft", radiansPerSecondLeft);
            table.put("targetRadiansPerSecondLeft", targetRadiansPerSecondLeft);
            table.put("ampsRight", ampsRight);
            table.put("ampsLeft", ampsLeft);
            table.put("voltsRight", voltsRight);
            table.put("voltsLeft", voltsLeft);
            table.put("tempRight", tempRight);
            table.put("tempLeft", tempLeft);

            // A subtable, thats only written to when in debug mode and never read from,
            // that provides some more human readable values
            if (ConstValues.DEBUG) {
                table.put("#Human/rpmRight", Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondRight));
                table.put("#Human/targetRPMRight",
                        Units.radiansPerSecondToRotationsPerMinute(targetRadiansPerSecondRight));
            }
        }

        @Override
        public void fromLog(LogTable table) {
            radiansPerSecondRight = table.get("radiansPerSecond", radiansPerSecondRight);
            targetRadiansPerSecondRight = table.get("targetRadiansPerSecond", targetRadiansPerSecondRight);
            radiansPerSecondLeft = table.get("radiansPerSecond", radiansPerSecondLeft);
            targetRadiansPerSecondLeft = table.get("targetRadiansPerSecond", targetRadiansPerSecondLeft);
            ampsRight = table.get("ampsRight", ampsRight);
            ampsLeft = table.get("ampsLeft", ampsLeft);
            voltsRight = table.get("voltsRight", voltsRight);
            voltsLeft = table.get("voltsLeft", voltsLeft);
            tempRight = table.get("tempRight", tempRight);
            tempLeft = table.get("tempLeft", tempLeft);
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
