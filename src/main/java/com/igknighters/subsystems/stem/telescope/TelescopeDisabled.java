package com.igknighters.subsystems.stem.telescope;

import org.littletonrobotics.junction.Logger;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;

public class TelescopeDisabled implements Telescope {
    private final TelescopeInputs inputs;
    final double slewRate = (0.5 / 50.0) * 0.75;

    public TelescopeDisabled() {
        inputs = new TelescopeInputs(StemPosition.STARTING.telescopeMeters);
    }

    @Override
    public double getTelescopeMeters() {
        return inputs.meters;
    }

    @Override
    public void setTelescopeMeters(double meters) {
        inputs.targetMeters = meters;
        inputs.meters = inputs.meters + MathUtil.clamp(meters - inputs.meters, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.volts = volts;
        double percentOut = volts / 12.0;
        inputs.metersPerSecond = slewRate * percentOut;
        inputs.meters += inputs.metersPerSecond;
    }

    @Override
    public boolean isFwdLimitSwitchHit() {
        inputs.isLimitFwdSwitchHit = inputs.meters >= kTelescope.MAX_METERS * 0.98;
        return inputs.isLimitFwdSwitchHit;
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        inputs.isLimitRevSwitchHit = inputs.meters <= kTelescope.MIN_METERS * 0.98;
        return inputs.isLimitRevSwitchHit;
    }

    @Override
    public boolean hasHomed() {
        inputs.isHomed = true;
        return inputs.isHomed;
    }

    @Override
    public void periodic() {
        Logger.processInputs("Stem/Telescope", inputs);
    }
}
