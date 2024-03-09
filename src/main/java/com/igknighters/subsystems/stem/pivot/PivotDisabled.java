package com.igknighters.subsystems.stem.pivot;

import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.util.MotionMagicProfileSim;

import edu.wpi.first.wpilibj.RobotController;

public class PivotDisabled extends Pivot {
    final double maxVelo = (2.37 / 50.0) * 0.75;
    final double maxAcc = 0.0001;
    final double maxJerk = 0.0001;

    MotionMagicProfileSim motionMagicProfileSim;
    double accelRadsPerSecond;

    public PivotDisabled() {
        super(StemPosition.STARTING.pivotRads);
        motionMagicProfileSim = new MotionMagicProfileSim(maxVelo, maxAcc, maxJerk, kPivot.MOTOR_TO_MECHANISM_RATIO);
        accelRadsPerSecond = 0.0;
    }

    @Override
    public double getPivotRadians() {
        return super.radians;
    }

    @Override
    public void setPivotRadians(double radians) {
        if (super.targetRadians != radians) 
            motionMagicProfileSim.setState(super.radians, radians, super.radiansPerSecond, accelRadsPerSecond);
        super.targetRadians = radians;

        super.radians = motionMagicProfileSim.calculatePosition();
        super.radiansPerSecond = motionMagicProfileSim.calculateVelocity();
        accelRadsPerSecond = motionMagicProfileSim.calculateAcceleration();
    }

    @Override
    public void setVoltageOut(double volts) {
        double percentOut = volts / RobotController.getBatteryVoltage();
        super.radiansPerSecond = maxVelo * percentOut;
        super.radians += super.radiansPerSecond;
    }
}
