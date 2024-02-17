package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.Component;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public interface Wrist extends Component {

    public static class WristInputs implements LoggableInputs {
        public double radians, targetRadians, radiansPerSecond = 0.0;
        public double volts = 0.0, amps = 0.0, temp = 0.0;

        public WristInputs(double startingRadians) {
            this.radians = startingRadians;
            this.targetRadians = startingRadians;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("radians", radians);
            table.put("targetRadians", targetRadians);
            table.put("radiansPerSecond", radiansPerSecond);
            table.put("volts", volts);
            table.put("amps", amps);
            table.put("temp", temp);
        }

        @Override
        public void fromLog(LogTable table) {
            radians = table.get("radians", radians);
            targetRadians = table.get("targetRadians", targetRadians);
            radiansPerSecond = table.get("radiansPerSecond", radiansPerSecond);
            volts = table.get("volts", volts);
            amps = table.get("amps", amps);
            temp = table.get("temp", temp);
        }
    }

    public void setWristRadians(Double radians);

    public double getWristRadians();

    /**
     * Moves the wrist to the target and returns if it has reached the target.
     * Meant to be used in a kind of polling loop to wait the mechanism to reach
     * the target.
     * 
     * @param radians      The target angle to move to
     * @param tolerancMult The multiplier to apply to the tolerance, higher mult
     *                     means more tolerance
     * @return If the mechanism has reached the target
     */
    default public boolean target(double radians, double tolerancMult) {
        this.setWristRadians(radians);
        return Math.abs(this.getWristRadians() - radians) < ConstValues.kStem.kWrist.TARGET_TOLERANCE * tolerancMult;
    }

    /**
     * Moves the wrist to the target and returns if it has reached the target.
     * Meant to be used in a kind of polling loop to wait the mechanism to reach
     * the target.
     * 
     * @param radians The target angle to move to
     * @return If the mechanism has reached the target
     */
    default public boolean target(double radians) {
        return target(radians, 1.0);
    }

    static double mechanismRadsToMotorRots(Double radians) {
        double clampedRadians = MathUtil.clamp(
                radians,
                kWrist.MIN_ANGLE,
                kWrist.MAX_ANGLE);

        if (clampedRadians != radians) {
            DriverStation.reportWarning(
                    "[Wrist] requested mechanism rads out of bounds: " + radians
                            + " (clamped to " + clampedRadians + ")",
                    false);
        }

        Logger.recordOutput("/Stem/Wrist/Math/ClampedRads", clampedRadians);

        double scopedRadians = (Math.PI / 2.0) - (clampedRadians - (Math.PI / 2.0));

        double beta = scopedRadians - kWrist.kDimensions.ANGLE_OFFSET;

        Logger.recordOutput("/Stem/Wrist/Math/Beta", beta);

        double leadscrewLength = Math.sqrt(
                Math.pow(kWrist.kDimensions.MOTOR_PIVOT_TO_WRIST_PIVOT, 2)
                        + Math.pow(kWrist.kDimensions.WRIST_PIVOT_TO_NUT, 2)
                        - (2 * kWrist.kDimensions.MOTOR_PIVOT_TO_WRIST_PIVOT * kWrist.kDimensions.WRIST_PIVOT_TO_NUT
                                * Math.cos(beta)));

        Logger.recordOutput("/Stem/Wrist/Math/LeadscrewInches", Units.metersToInches(leadscrewLength));

        return Units.metersToInches(leadscrewLength) * 16.0;
    }

    static double mechanismRadsToMotorRads(Double radians) {
        return Units.rotationsToRadians(mechanismRadsToMotorRots(radians));
    }

    static double motorRotsToMechanismRads(double motorRots) {
        double leadscrewLength = Units.inchesToMeters(motorRots / 16.0);

        double beta = Math.acos(
                (Math.pow(kWrist.kDimensions.MOTOR_PIVOT_TO_WRIST_PIVOT, 2)
                        + Math.pow(kWrist.kDimensions.WRIST_PIVOT_TO_NUT, 2)
                        - Math.pow(leadscrewLength, 2))
                        / (2 * kWrist.kDimensions.MOTOR_PIVOT_TO_WRIST_PIVOT * kWrist.kDimensions.WRIST_PIVOT_TO_NUT));

        double scopedRadians = beta + kWrist.kDimensions.ANGLE_OFFSET;

        double clampedRadians = (Math.PI / 2.0) - (scopedRadians - (Math.PI / 2.0));

        return clampedRadians;
    }

    static double motorRadsToMechanismRads(double motorRads) {
        return motorRotsToMechanismRads(Units.radiansToRotations(motorRads));
    }
}
