package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.Logger;
import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;

public class WristDisabled implements Wrist {
    private final WristInputs inputs;
    final double slewRate = (4.3 / 50.0) * 0.75;

    public WristDisabled() {
        inputs = new WristInputs(StemPosition.STARTING.wristRads);
    }

    @Override
    public double getWristRadians() {
        return inputs.radians;
    }

    @Override
    public void setWristRadians(double radians) {
        inputs.targetRadians = radians;
        inputs.radians = inputs.radians + MathUtil.clamp(radians - inputs.radians, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.volts = volts;
        double percentOut = volts / 12.0;
        inputs.radiansPerSecond = slewRate * percentOut;
        inputs.radians += inputs.radiansPerSecond;
    }

    @Override
    public void periodic() {
        Logger.processInputs("Stem/Wrist", inputs);
    }

    @Override
    public void seedWrist() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'seedWrist'");
    }
}
