package com.igknighters.subsystems.swerve.gyro;

import com.igknighters.subsystems.Component;

import monologue.Annotations.Log;

public abstract class Gyro extends Component {
    @Log public double pitchRads = 0.0;
    @Log public double rollRads = 0.0;
    @Log public double yawRads = 0.0;
    @Log public double pitchVelRadsPerSec = 0.0;
    @Log public double rollVelRadsPerSec = 0.0;
    @Log public double yawVelRadsPerSec = 0.0;

    @Override
    public String getOverrideName() {
        return "Gyro";
    }

    public double getPitchRads() {
        return pitchRads;
    }

    public double getRollRads() {
        return rollRads;
    }

    public double getYawRads() {
        return yawRads;
    }

    public abstract void setYawRads(double yawRads);
}
