package com.igknighters.subsystems.stem.pivot;

import org.littletonrobotics.junction.Logger;
import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;

public class PivotDisabled implements Pivot {
    private final PivotInputs inputs;
    final double slewRate = (2.37 / 50.0) * 0.75;

    public PivotDisabled() {
        inputs = new PivotInputs(StemPosition.STARTING.pivotRads);
    }

    @Override
    public double getPivotRadians() {
        return inputs.radians;
    }

    @Override
    public void setPivotRadians(double radians) {
        // var clampedTarget = MathUtil.clamp(radians, kPivot.MIN_ANGLE, kPivot.MAX_ANGLE);
        inputs.radians = inputs.radians + MathUtil.clamp(radians - inputs.radians, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        double percentOut = volts / 12.0;
        inputs.radiansPerSecond = slewRate * percentOut;
        inputs.radians += inputs.radiansPerSecond;
    }

    @Override
    public void periodic() {
        Logger.processInputs("Stem/Pivot", inputs);
    }
}
