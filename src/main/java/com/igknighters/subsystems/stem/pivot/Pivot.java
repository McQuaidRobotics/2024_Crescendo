package com.igknighters.subsystems.stem.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;

public interface Pivot extends Component {

    public static class PivotInputs implements LoggableInputs {
        public double degrees, targetDegrees, degreesPerSecond = 0.0;
        public double volts = 0.0;
        public double leftAmps = 0.0, rightAmps = 0.0;
        public double leftTemp = 0.0, rightTemp = 0.0;
        public boolean isLimitSwitchHit = false;

        public PivotInputs(double startingDegrees) {
            this.degrees = startingDegrees;
            this.targetDegrees = startingDegrees;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("Stem/Pivot/degrees", degrees);
            table.put("Stem/Pivot/targetDegrees", targetDegrees);
            table.put("Stem/Pivot/degreesPerSecond", degreesPerSecond);
            table.put("Stem/Pivot/volts", volts);
            table.put("Stem/Pivot/leftAmps", leftAmps);
            table.put("Stem/Pivot/rightAmps", rightAmps);
            table.put("Stem/Pivot/leftTemp", leftTemp);
            table.put("Stem/Pivot/rightTemp", rightTemp);
            table.put("Stem/Pivot/isLimitSwitchHit", isLimitSwitchHit);
        }

        @Override
        public void fromLog(LogTable table) {
            degrees = table.get("Stem/Pivot/degrees", degrees);
            targetDegrees = table.get("Stem/Pivot/targetDegrees", targetDegrees);
            degreesPerSecond = table.get("Stem/Pivot/degreesPerSecond", degreesPerSecond);
            volts = table.get("Stem/Pivot/volts", volts);
            leftAmps = table.get("Stem/Pivot/leftAmps", leftAmps);
            rightAmps = table.get("Stem/Pivot/rightAmps", rightAmps);
            leftTemp = table.get("Stem/Pivot/leftTemp", leftTemp);
            rightTemp = table.get("Stem/Pivot/rightTemp", rightTemp);
            isLimitSwitchHit = table.get("Stem/Pivot/isLimitSwitchHit", isLimitSwitchHit);
        }
    }


}
