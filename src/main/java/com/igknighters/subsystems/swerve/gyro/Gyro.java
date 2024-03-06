package com.igknighters.subsystems.swerve.gyro;

import com.igknighters.subsystems.Component;

public abstract class Gyro extends Component {
    public double pitchRads = 0.0;
    public double rollRads = 0.0;
    public double yawRads = 0.0;
    public double pitchVelRadsPerSec = 0.0;
    public double rollVelRadsPerSec = 0.0;
    public double yawVelRadsPerSec = 0.0;

    public double getPitchRads() {
        return pitchRads;
    }

    public double getRollRads() {
        return pitchRads;
    }

    public double getYawRads() {
        return pitchRads;
    }

    public abstract void setYawRads(double yawRads);
}
