package com.igknighters.subsystems.umbrella.shooter;

import com.igknighters.subsystems.Component;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import monologue.Annotations.Log;

public abstract class Shooter extends Component {

    @Log protected double radiansPerSecondRight = 0.0;
    @Log protected double targetRadiansPerSecondRight = 0.0;
    @Log protected double radiansPerSecondLeft = 0.0;
    @Log protected double targetRadiansPerSecondLeft = 0.0;
    @Log protected double voltsRight = 0.0;
    @Log protected double voltsLeft = 0.0;
    @Log protected double ampsRight = 0.0;
    @Log protected double ampsLeft = 0.0;
    @Log protected double tempRight = 0.0;
    @Log protected double tempLeft = 0.0;
    @Log protected double shooterRightRPM = Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondRight);
    @Log protected double shooterLeftRPM = Units.radiansPerSecondToRotationsPerMinute(radiansPerSecondLeft);
    @Log protected double targetShooterRightRPM = Units.radiansPerSecondToRotationsPerMinute(targetRadiansPerSecondRight);
    @Log protected double targetShooterLeftRPM = Units.radiansPerSecondToRotationsPerMinute(targetRadiansPerSecondLeft);

    @Override
    public String getOverrideName() {
        return "Shooter";
    }

    /**
     * @return The rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public abstract double getSpeed();

    /**
     * @return The target rotational speed of the {@code Shooter} flywheel in Rad/S
     */
    public abstract double getTargetSpeed();

    /**
     * Runs the {@code Shooter} in closed loop at the specified speed
     * 
     * @param speedRadPerSec The speed in Rad/S to spin the flywheel at
     */
    public abstract void setSpeed(double speedRadPerSec);

    /**
     * Runs the mechanism in open loop at the specified voltage
     * 
     * @param volts The specified volts: [-12.0 .. 12.0]
     */
    public abstract void setVoltageOut(double volts);

    /**
     * 
     * @param rpm The rpm of the shooter
     * @return 
     */
    public static double rpmToMps(double rpm) {
        final double LOW_END_RPM = 4500.0;
        final double LOW_END_MPS = 11.7;
        final double HIGH_END_RPM = 8000.0;
        final double HIGH_END_MPS = 15.2;
        final double DIFF_RPM = HIGH_END_RPM - LOW_END_RPM;
        final double DIFF_MPS = HIGH_END_MPS - LOW_END_MPS;

        double clamped = MathUtil.clamp(rpm, LOW_END_RPM, HIGH_END_RPM);
        double t = (clamped - LOW_END_RPM) / DIFF_RPM;
        return (t * DIFF_MPS) + LOW_END_MPS;
    }
}
