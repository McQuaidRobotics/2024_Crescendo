package com.igknighters.subsystems.swerve.gyro;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.Component;

import edu.wpi.first.math.util.Units;

public interface Gyro extends Component {
    public static class GyroInputs implements LoggableInputs {
        public double pitchRads = 0.0, rollRads = 0.0, yawRads = 0.0;
        public double pitchVelRadsPerSec = 0.0, rollVelRadsPerSec = 0.0, yawVelRadsPerSec = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("PitchRads", pitchRads);
            table.put("RollRads", rollRads);
            table.put("GyroYawRads", yawRads);
            table.put("PitchVelRadsPerSec", pitchVelRadsPerSec);
            table.put("RollVelRadsPerSec", rollVelRadsPerSec);
            table.put("GyroYawVelRadsPerSec", yawVelRadsPerSec);

            if (ConstValues.DEBUG) {
                table.put("#Human/PitchDegrees", Units.radiansToDegrees(pitchRads));
                table.put("#Human/RollDegrees", Units.radiansToDegrees(rollRads));
                table.put("#Human/YawDegrees", Units.radiansToDegrees(yawRads));
            }
        }

        @Override
        public void fromLog(LogTable table) {
            pitchRads = table.get("PitchRads", pitchRads);
            rollRads = table.get("RollRads", rollRads);
            yawRads = table.get("YawRads", yawRads);
            pitchVelRadsPerSec = table.get("PitchVelRadsPerSec", pitchVelRadsPerSec);
            rollVelRadsPerSec = table.get("RollVelRadsPerSec", rollVelRadsPerSec);
            yawVelRadsPerSec = table.get("YawVelRadsPerSec", yawVelRadsPerSec);
        }
    }

    public double getPitchRads();

    public double getRollRads();

    public double getYawRads();

    public void setYawRads(double yawRads);
}
