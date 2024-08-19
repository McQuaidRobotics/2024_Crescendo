package com.igknighters.subsystems.stem.telescope;

import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;

public class TelescopeDisabled extends Telescope {
    final double slewRate = (0.5 / 50.0) * 0.75;

    public TelescopeDisabled() {
        super(StemPosition.STARTING.telescopeMeters);
    }

    @Override
    public void gotoPosition(double meters) {
        super.targetMeters = meters;
        super.meters = super.meters + MathUtil.clamp(meters - super.meters, -slewRate, slewRate);
    }

    @Override
    public void setVoltageOut(double volts) {
        super.volts = volts;
        double percentOut = volts / RobotController.getBatteryVoltage();
        super.metersPerSecond = slewRate * percentOut;
        super.meters += super.metersPerSecond;
    }

    @Override
    public boolean isFwdLimitSwitchHit() {
        super.isLimitFwdSwitchHit = super.meters >= kTelescope.MAX_METERS * 0.98;
        return super.isLimitFwdSwitchHit;
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        super.isLimitRevSwitchHit = super.meters <= kTelescope.MIN_METERS * 0.98;
        return super.isLimitRevSwitchHit;
    }

    @Override
    public boolean hasHomed() {
        super.isHomed = true;
        return super.isHomed;
    }
}
