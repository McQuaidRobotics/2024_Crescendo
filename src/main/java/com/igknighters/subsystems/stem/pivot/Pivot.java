package com.igknighters.subsystems.stem.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

public interface Pivot extends Component {

    public static class PivotInputs implements LoggableInputs {
        public double radians, targetRadians, radiansPerSecond = 0.0;
        public double volts = 0.0;
        public double leftAmps = 0.0, rightAmps = 0.0;
        public double leftTemp = 0.0, rightTemp = 0.0;
        public double gyroPitchRadians = 0.0;
        public boolean isLimitSwitchHit = false;

        public PivotInputs(double startingRadians) {
            this.radians = startingRadians;
            this.targetRadians = startingRadians;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("radians", radians);
            table.put("targetRadians", targetRadians);
            table.put("radiansPerSecond", radiansPerSecond);
            table.put("volts", volts);
            table.put("leftAmps", leftAmps);
            table.put("rightAmps", rightAmps);
            table.put("leftTemp", leftTemp);
            table.put("rightTemp", rightTemp);
            table.put("gyroPitchRadians", gyroPitchRadians);
            table.put("isLimitSwitchHit", isLimitSwitchHit);

            // A subtable, thats only written to when in debug mode and never read from,
            // that provides some more human readable values
            if (ConstValues.DEBUG) {
                table.put("#Human/degrees", Math.toDegrees(radians));
                table.put("#Human/targetDegrees", Math.toDegrees(targetRadians));
                table.put("#Human/degreesPerSecond", Math.toDegrees(radiansPerSecond));
                table.put("#Human/watts", volts * (leftAmps + rightAmps));
            }
        }

        @Override
        public void fromLog(LogTable table) {
            radians = table.get("radians", radians);
            targetRadians = table.get("targetRadians", targetRadians);
            radiansPerSecond = table.get("radiansPerSecond", radiansPerSecond);
            volts = table.get("volts", volts);
            leftAmps = table.get("leftAmps", leftAmps);
            rightAmps = table.get("rightAmps", rightAmps);
            leftTemp = table.get("leftTemp", leftTemp);
            rightTemp = table.get("rightTemp", rightTemp);
            gyroPitchRadians = table.get("gyroPitchRadians", gyroPitchRadians);
            isLimitSwitchHit = table.get("isLimitSwitchHit", isLimitSwitchHit);
        }
    }
    public void setPivotRadians(double radians);

    /**
     * @return the current angle of the mechanism
     */
    public double getPivotRadians();

}
